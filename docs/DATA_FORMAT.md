# Recorded diagnostic data

`record_curves` subscribes to `/hat_x_topic` and writes 38 comma-separated fields per sample, retaining the original `data[0]` through `data[37]` header. The file contains numeric diagnostic messages, not recovered historical results.

| Indices | Meaning |
| --- | --- |
| 0, 1 | X estimated relative-position error and error velocity |
| 2, 3 | X commanded UAV acceleration a_q and most recently released visual tracking-error measurement |
| 4, 5 | UAV motion-capture X position and velocity |
| 6, 7 | Target motion-capture X position and velocity |
| 8, 9 | X predicted state associated with the applied buffer entry |
| 10, 11 | X estimated disturbance C_w w_hat; joint first-input event error |
| 12-21 | Y equivalents of fields 0-9 |
| 22 | Software DoS state: 1 attacked, 0 normal |
| 23 | Y estimated disturbance C_w w_hat |
| 24-33 | Z equivalents of fields 0-9 |
| 34, 35 | Z estimated disturbance C_w w_hat; successful packet reception flag |
| 36 | Zero-based fast control step counter |
| 37 | Duration of this control tick through command publication, in seconds |

The column count remains 38, but the mathematical meanings above replace the old reconstructed controller's meanings. The low-pass visual filter was removed. Field 2 (and its Y/Z equivalents) is the limited UAV acceleration command; the core uses the opposite-sign virtual input u = -a_q. Field 36 is a step counter, not a hardware timestamp. Field 37 now includes the core calculation and command publication, but excludes the subsequent log publication and external flight-controller processing.

The additional topic /asp_phbsc/diagnostics publishes 15 fields:

| Index | Meaning |
| --- | --- |
| 0, 1 | ROS tick timestamp, zero-based step |
| 2, 3 | Real sample used, candidate packet rebuilt |
| 4, 5 | Transmission attempted, packet received |
| 6, 7 | Software DoS active, acceleration command saturated |
| 8 | Accepted measurement age in seconds, or -1 if no new sample |
| 9, 10 | Joint first-input event error, event threshold |
| 11-13 | Actual commanded virtual inputs u_x, u_y, u_z after adapter limits |
| 14 | Fast-step gap from the last valid sample: 0 initially, -1 without a new sample, otherwise a positive multiple of M |

The 38-column CSV recorder does not automatically save this separate diagnostics topic. The measured age documents timestamp handling; it is not evidence that camera and flight-controller clocks have been calibrated.

The recorder selects a new timestamped file by default and accepts a private ROS parameter `output_path`. It rejects an existing output file instead of overwriting it. Set a distinct output file per run and keep the selected case and any synthetic attack table with the recorded data.
