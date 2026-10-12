#include "visual_target.h"

#include <utility>
#include <iostream>
#include <cstring>
#include <cmath>
#include <stdexcept>
using namespace std;

// 相机坐标系到无人机机体系（IMU）的小偏移
static const Eigen::Vector3d POS_OFFSET(-0.023, -0.063, 0.03297);

ObjectDetector::ObjectDetector(ros::NodeHandle &nh)
    : nh_(nh),
      lost_target(true),
      desired_yaw(0.0),
      clockwise(true)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("processing_rate_hz", processing_rate_hz_, 20.0);
  if (private_nh.hasParam("measurement_period_s"))
  {
    double measurement_period_s;
    if (!private_nh.getParam("measurement_period_s", measurement_period_s)
        || !std::isfinite(measurement_period_s) || measurement_period_s <= 0.0)
    {
      throw std::invalid_argument("measurement_period_s must be finite and greater than zero");
    }
    // Prefer the shared measurement period while retaining the legacy rate parameter.
    processing_rate_hz_ = 1.0 / measurement_period_s;
  }
  if (!std::isfinite(processing_rate_hz_) || processing_rate_hz_ <= 0.0)
  {
    throw std::invalid_argument("processing_rate_hz must be finite and greater than zero");
  }
  // The processing/measurement loop is independent of the raw 60 fps color stream.
  // No artificial measurement delay is introduced here.
  ROS_INFO_STREAM("Visual processing rate: " << processing_rate_hz_
                  << " Hz; raw color stream: " << fps_ << " fps");

  // ----------------- 1. RealSense 原生 pipeline 初始化 -----------------
  cfg_.disable_stream(RS2_STREAM_DEPTH);
  cfg_.disable_stream(RS2_STREAM_INFRARED);
  cfg_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);

  try
  {
    profile_ = pipe_.start(cfg_);

    // Retain the original device queue setting (10 frames).
    rs2::device dev = profile_.get_device();
    for (rs2::sensor s : dev.query_sensors())
    {
      if (s.supports(RS2_OPTION_GLOBAL_TIME_ENABLED))
      {
        // 尽量让 timestamp 对齐到 host time（若设备支持）
        try
        {
          s.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
        }
        catch (...)
        {
        }
      }
      if (s.supports(RS2_OPTION_FRAMES_QUEUE_SIZE))
      {
        s.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 10.0f);
      }
    }

    // 取 color stream 内参
    auto color_stream = profile_.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    rs2_intrinsics in = color_stream.get_intrinsics();

    // ViSP 相机模型（无畸变）
    cam.initPersProjWithoutDistortion(in.fx, in.fy, in.ppx, in.ppy);
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("ObjectDetector: Failed to start RealSense pipeline: " << e.what());
    throw;
  }

  // ----------------- 3. AprilTag 检测器配置 -----------------
  tag_detector.setAprilTagQuadDecimate(2);
  tag_detector.setAprilTagPoseEstimationMethod(vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS);
  tag_detector.setAprilTagNbThreads(4);
  tag_detector.setAprilTagFamily(vpDetectorAprilTag::TAG_36h11);
  tag_detector.setZAlignedWithCameraAxis(false);

  // ----------------- 4. 位姿相关变量初始化 -----------------
  Position_before = Eigen::Vector3d::Zero();
  Position_after = Eigen::Vector3d::Zero();

  R_i2c << 0.0114, -0.0215, 0.997,
      -0.997, 0.0182, 0.0118,
      -0.0184, -0.996, -0.0213;

  R_tagfix << 0, -1, 0,
      1, 0, 0,
      0, 0, 1;

  {
    Eigen::Quaterniond q_i2c(R_i2c);
    R_i2c = q_i2c.normalized().toRotationMatrix();
  }

  R_w2c = R_i2c;
  R_c2a = Eigen::Matrix3d::Identity();
  R_w2a = Eigen::Matrix3d::Identity();

  // ----------------- 5. ROS 通信 -----------------
  point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/point_with_fixed_delay", 1, true);

  odom_sub_ = nh_.subscribe<sensor_msgs::Imu>(
      "/mavros/imu/data", 10, // 队列建议大一点，方便插值
      &ObjectDetector::odomCallback, this,
      ros::TransportHints().tcpNoDelay());

  image_pub_ = nh_.advertise<sensor_msgs::Image>("/camera/image", 1);
}

ObjectDetector::~ObjectDetector()
{
  try
  {
    pipe_.stop();
  }
  catch (...)
  {
  }
}

void ObjectDetector::start()
{
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate rate(processing_rate_hz_);
  while (ros::ok())
  {
    // double start_time = ros::Time::now().toSec();
    processImages();
    // double end_time = ros::Time::now().toSec();
    // cout << 1000 * (end_time - start_time) << endl;
    rate.sleep();
  }
}

void ObjectDetector::odomCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  Eigen::Quaterniond q(
      msg->orientation.w,
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z);

  if (msg->header.stamp.isZero() || !q.coeffs().allFinite() || q.squaredNorm() < 1e-12)
  {
    ROS_WARN_THROTTLE(2.0, "Discarded an invalid IMU attitude or timestamp.");
    return;
  }
  q.normalize();

  // 新增：缓存带时间戳的姿态，给图像对齐用
  {
    std::lock_guard<std::mutex> lk(imu_mtx_);
    // A time reset must not leave a nonmonotone queue for interpolation.
    if (!imu_buf_.empty() && msg->header.stamp <= imu_buf_.back().t)
      imu_buf_.clear();
    imu_buf_.push_back({msg->header.stamp, q});

    while (!imu_buf_.empty() &&
           (imu_buf_.back().t - imu_buf_.front().t).toSec() > imu_buf_span_sec_)
    {
      imu_buf_.pop_front();
    }
  }
}

bool ObjectDetector::getQuatAt(const ros::Time &t, Eigen::Quaterniond &q_out)
{
  std::lock_guard<std::mutex> lk(imu_mtx_);
  if (imu_buf_.empty())
    return false;

  auto it = std::lower_bound(
      imu_buf_.begin(), imu_buf_.end(), t,
      [](const ImuSample &a, const ros::Time &tt)
      { return a.t < tt; });

  if (it == imu_buf_.begin())
  {
    if ((it->t - t).toSec() > 0.02) return false;
    q_out = it->q;
    return true;
  }
  if (it == imu_buf_.end())
  {
    if ((t - imu_buf_.back().t).toSec() > 0.02) return false;
    q_out = imu_buf_.back().q;
    return true;
  }

  const auto &s1 = *(it - 1);
  const auto &s2 = *it;

  double denom = (s2.t - s1.t).toSec();
  if (denom <= 1e-6)
  {
    q_out = s2.q;
    return true;
  }

  double a = (t - s1.t).toSec() / denom;
  a = std::min(1.0, std::max(0.0, a));
  q_out = s1.q.slerp(a, s2.q);
  return true;
}

ros::Time ObjectDetector::frameToRosTime(const rs2::frame &f)
{
  ros::Time t;
  const double ts_ms = f.get_timestamp();
  const auto dom = f.get_frame_timestamp_domain();
  // A fixed-delay observer needs a capture stamp in the ROS clock domain.
  // A host arrival stamp or an arbitrary one-time device-clock offset cannot
  // establish that timing, so those old fallbacks are deliberately rejected.
  if (ros::Time::isSimTime() ||
      (dom != RS2_TIMESTAMP_DOMAIN_SYSTEM_TIME && dom != RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME))
    throw std::runtime_error("Camera capture clock is not synchronized to ROS wall time");
  const double stamp_s = ts_ms * 1e-3;
  const double age_s = ros::Time::now().toSec() - stamp_s;
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0 || age_s < 0.0 || age_s > 1.0)
    throw std::runtime_error("Invalid or stale camera capture timestamp");
  t.fromSec(stamp_s);
  return t;
}

void ObjectDetector::publishDetectionResult(ros::Time &ts, bool is_fault)
{
  static std_msgs::Float64MultiArray msg;
  if (msg.data.size() != 6)
    msg.data.resize(6);

  msg.data[0] = Position_after.x();
  msg.data[1] = Position_after.y();
  msg.data[2] = Position_after.z();
  msg.data[3] = ts.toSec();
  msg.data[4] = static_cast<double>(is_fault || lost_target);
  msg.data[5] = desired_yaw;

  point_pub_.publish(msg);

  if (is_fault)
  {
    ROS_WARN_STREAM("Fault detected, skipping processing.");
  }
}

// 你当前实现是“直接发布”（保持原样）
void ObjectDetector::baseProcess(ros::Time ts, bool is_fault)
{
  if (!ros::ok())
    return;
  publishDetectionResult(ts, is_fault);
}

// 处理一轮：尽量拿“最新帧”
void ObjectDetector::processImages()
{
  ros::Time image_timestamp;
  bool fault_detected = false;

  try
  {
    // 1) 抽干队列：只保留最后一帧（最新帧）
    rs2::frameset fs, latest;
    bool has = false;
    if (pipe_.poll_for_frames(&fs))
    {
      latest = fs;
      has = true;
    }

    if (!has)
    {
      // 本周期没有新帧：发布 fault
      image_timestamp = ros::Time::now();
      fault_detected = true;
      lost_target = true;
      baseProcess(image_timestamp, fault_detected);
      cout << "here" << endl;
      return;
    }

    rs2::video_frame color = latest.get_color_frame();
    if (!color)
    {
      image_timestamp = ros::Time::now();
      fault_detected = true;
      lost_target = true;
      baseProcess(image_timestamp, fault_detected);
      return;
    }

    // 2) 图像时间戳（尽量真实）
    image_timestamp = frameToRosTime(color);

    // 3) RGB8 -> Gray
    const int w = color.get_width();
    const int h = color.get_height();
    cv::Mat rgb(h, w, CV_8UC3, (void *)color.get_data(), (size_t)color.get_stride_in_bytes());
    cv::Mat gray;
    cv::cvtColor(rgb, gray, cv::COLOR_RGB2GRAY);

    // 4) Gray -> vpImage<unsigned char>
    vpImage<unsigned char> I((unsigned int)h, (unsigned int)w);
    std::memcpy(I.bitmap, gray.data, (size_t)(w * h));

    // 5) 用图像时间戳插值 IMU 姿态，算 R_w2c
    Eigen::Quaterniond q_img;
    if (!getQuatAt(image_timestamp, q_img))
      throw std::runtime_error("No time-aligned IMU attitude is available for this image");
    const Eigen::Matrix3d R_w2i_img = q_img.toRotationMatrix();
    R_w2c = R_w2i_img * R_i2c;

    std_msgs::Header header;
    header.stamp = image_timestamp;
    sensor_msgs::ImagePtr img_msg =
        cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
    image_pub_.publish(img_msg);

    // 8) AprilTag 检测
    static std::vector<vpHomogeneousMatrix> cMo_vec;
    cMo_vec.clear();
    cMo_vec.reserve(1);

    tag_detector.detect(I, 0.08, cam, cMo_vec);
    fault_detected = (cMo_vec.size() != 1);

    if (!fault_detected)
    {
      // 相机坐标系下 Tag 位置
      Position_before[0] = cMo_vec[0][0][3];
      Position_before[1] = cMo_vec[0][1][3];
      Position_before[2] = cMo_vec[0][2][3];
      Position_before += POS_OFFSET;

      // 世界系
      Position_after = R_w2c * Position_before;

      // 相机到 AprilTag 的旋转
      vpRotationMatrix R_c2a_vp = cMo_vec[0].getRotationMatrix();
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          R_c2a(i, j) = R_c2a_vp[i][j];
      // 拼旋转链
      R_w2a = R_w2c * R_c2a * R_tagfix;
      desired_yaw = std::atan2(R_w2a(1, 0), R_w2a(0, 0));

      static int count_ = 0;
      if (++count_ % 30 == 0)
      {
        std::cout << "pos:     " << Position_after.transpose() << std::endl;
        std::cout << "yaw:     " << desired_yaw << std::endl;
        count_ = 0;
      }
      lost_target = false;
    }
    else
    {
      lost_target = true;
    }
  }
  catch (const std::exception &e)
  {
    ROS_ERROR_STREAM("Processing error: " << e.what());
    image_timestamp = ros::Time::now();
    fault_detected = true;
    lost_target = true;
  }

  baseProcess(image_timestamp, fault_detected);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_target_node");
  ros::NodeHandle nh;

  try
  {
    ObjectDetector detector(nh);
    detector.start();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL_STREAM("Visual target node failed: " << e.what());
    return 1;
  }
  return 0;
}
