# Implementation status and provenance

The current ASP-PHBSC core is an implementation of the manuscript's printed discrete equations and packet algorithms. It is not a recovered historical flight executable and does not claim to regenerate the manuscript's figures or RMSE values. The experimental disturbance model and complete augmented observer gain remain unspecified, as requested by the author; the ROS node refuses to start until they are supplied.

## Aligned configuration

Section VI supplies the control and visual measurement periods, K, L, event thresholds, delay compensation length, maximum event interval, prediction parameter, target radius and period, wind condition, and maximum consecutive DoS lengths. These values are separated into common and case-specific YAML files.

DoS attack start times, normal communication intervals, measured processing/network latency, and the raw flight data have not been recovered. The optional synthetic attack table is a new demonstration input. Physical target speed and wind are experiment metadata, not software actuator commands.

## Implemented equation mapping

See [the equation and timing guide](PAPER_IMPLEMENTATION.md). The core includes the augmented disturbance state, matrix-exponential discretization of A, B and L, delayed observer and input-history state prediction, negative virtual feedback with disturbance compensation, N_p + 1 input packets, zero-filled shifts, and first-input event triggering timed from the last transmission attempt. Three axes share one packet and event clock. Candidate packets are rebuilt only on real sampled-data arrival, except for the explicitly assumed initial packet.

The ROS adapter uses one fast clock and translates the paper's virtual error-system input u into UAV acceleration a_q = -u. Hardware acceleration limits are applied outside the mathematical core, and the resulting commanded input is fed back into the observer's history. This history represents the sent command, not a measurement proving perfect low-level acceleration tracking.

## Unresolved manuscript information and implementation limits

1. **Missing experimental model:** the experimental physical state already has two components per axis. An augmented observer therefore needs more than two gain components. Section VI only gives L = [-0.6, -6.6], and does not specify the UAV A_w, C_w or full augmented L. The configuration fields disturbance_dynamics, disturbance_output and augmented_observer_gain are intentionally empty. Test fixtures use explicitly chosen synthetic models; they are never loaded as flight parameters.
2. **Printed predictor convention:** the discrete output predictor is implemented exactly as printed, y_i^p = C A_d z_i + C B_d u_(i-n_tau). Its right-hand side is a one-step prediction although the displayed subscript is i. The current estimate uses z_i; a measurement at step i changes z_(i+1). This convention is explicit in the guide and tests; the manuscript itself has not been edited.
3. **Packet deployment:** controller and actuator-buffer logic have separate states but run in one process on one tick. The software channel gates delivery of newly triggered packets while the actuator buffer continues shifting. This implements the discrete reception logic; it is not a deployed receiver on the Pixhawk or a real ROS/MAVLink network attack.
4. **Reception guarantee:** failed transmission attempts advance the event clock, as in the printed ET definition. The configured horizon satisfies N_p >= N_e + N_ca. That inequality alone does not prove a successful-reception bound for every attack trace: repeated short attacks at event instants can block successive attempts. Buffer exhaustion uses the printed zero-filled shift; no tail hold is hidden in the algorithm.
5. **Timing and hardware:** timestamps, acquisition latency, ROS scheduling, independent FCU localization and actual low-level tracking still require platform validation. The default launch no longer routes motion capture into flight feedback; motion capture supplies diagnostic ground truth. Firmware EKF configuration must be checked separately before claiming that the complete physical platform excludes motion capture feedback.
6. **Missing comparisons and data:** PBESO, ETSC, historical attack traces and figure-generation flight data have not been recovered. They are not generated or relabeled here.

The mathematical implementation and the unavailable historical experimental settings are separate. See [validation](VALIDATION.md) for exactly which checks were run.
