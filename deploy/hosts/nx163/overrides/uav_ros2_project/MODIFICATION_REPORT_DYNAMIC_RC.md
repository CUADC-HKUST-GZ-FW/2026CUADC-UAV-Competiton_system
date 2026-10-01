# Dynamic RC / Dynamic R modification report

## Scope

This change replaces the production dynamic-R placeholder with an AB multi-sample prediction model while preserving the verified full-mission update architecture.

Unchanged mission layout:

- A = seq 5
- U = seq 6
- R = seq 7
- DO_SET_SERVO = seq 8
- D = seq 9
- B and C remain virtual points

The second full mission Push -> Pull -> Verify -> mission-progress reconciliation path is not redesigned.

## Changed files

1. `src/uav_fcu_interface/uav_fcu_interface/fcu_interface_mavros_node.py`
   - subscribes to `/mavros/global_position/rel_alt`
   - enriches the existing AB GPS history with VFR motion state and relative altitude
   - freezes the last prediction window at B crossing
   - estimates forward speed with a recency-weighted mean
   - estimates vertical speed with a weighted linear trend; automatically falls back to weighted mean if the trend quality is poor
   - prevents a damping vertical trend from being extrapolated through zero into a fake climb/descent reversal
   - iteratively solves RC and generates R only along the fixed attack line
   - retains explicit TEST fixed-offset mode
   - retains R-safe fallback on invalid/stale/unsafe prediction inputs
   - logs prediction inputs, each RC iteration, and predicted-vs-actual state at R

2. `src/uav_bringup/uav_bringup/launch_params.py`
   - enables the prediction path by default (`dynamic_r_test_mode=false`)
   - adds centralized prediction parameters
   - keeps `dynamic_r_release_delay_sec=0.0` for now

3. `src/uav_bringup/uav_bringup/flight_summary_logger_node.py`
   - records and renders the new prediction events in the human-readable summary and ABURCD metrics stream

4. `src/uav_fcu_interface/uav_fcu_interface/test_abcdr_mission.py`
   - updates the non-ROS fixture for the new parameters/state
   - adds level-flight, descending-flight, trend-to-level, bad-fit fallback, stale-altitude, and RC-bound tests

## Prediction model

At B crossing, the model freezes the most recent AB samples (default 1.2 s, minimum 5 samples).

Forward velocity:

`v_forward = groundspeed * cos(heading_error)`

The final forward velocity estimate is a recency-weighted mean.

Vertical velocity:

A weighted linear fit estimates `vz_B` and vertical acceleration/trend `az`.

If fit RMSE or acceleration magnitude is outside configured limits, the model falls back to the recency-weighted mean vertical speed with `az=0`.

If the trend is damping climb/descent toward zero and would cross zero before R, the prediction levels at `vz=0` rather than extrapolating into an artificial reversal.

Height source:

`/mavros/global_position/rel_alt`

The current first-version assumption is that target ground elevation is approximately the same as Home ground elevation. Global GPS altitude is not used as ballistic height.

Ballistic model (first version):

`H_R + vz_R*t - 0.5*g*t^2 = 0`

Positive root gives `t_fall`.

`RC = v_forward * (t_fall + release_delay)`

`release_delay` is currently configured as 0.0 s but the parameter/interface is already present.

RC is solved by fixed-point iteration because the predicted R state depends on B->R travel time, which itself depends on RC.

## New shared parameters

Defaults in `launch_params.py`:

- `dynamic_r_prediction_window_sec = 1.2`
- `dynamic_r_min_prediction_samples = 5`
- `dynamic_r_min_prediction_span_sec = 0.4`
- `dynamic_r_vz_fit_max_rmse_mps = 0.8`
- `dynamic_r_max_abs_vertical_accel_mps2 = 3.0`
- `dynamic_r_min_rc_m = 20.0`
- `dynamic_r_min_u_r_distance_m = 5.0`
- `dynamic_r_max_iterations = 4`
- `dynamic_r_convergence_m = 0.5`
- `dynamic_r_release_delay_sec = 0.0`
- `dynamic_r_prediction_shadow_mode = false`

The old fixed-offset path remains available with:

`dynamic_r_test_mode=true`

## Fallback behavior

The current safe R is retained if, among other cases:

- prediction samples are insufficient
- sample span is too short
- VFR motion samples are invalid
- rel_alt is missing, stale, non-finite, or non-positive
- forward speed is invalid
- attack geometry is invalid
- predicted release state is invalid
- ballistic solution is invalid
- RC leaves configured bounds
- RC iteration does not converge
- candidate fails U-C geometric validation

## New useful log events

- `AB_PREDICTION_INPUT`
- `R_PREDICTION_ITERATION`
- `R_CALC_DONE`
- `R_CALC_SHADOW` (when shadow mode is enabled)
- `RELEASE_STATE_ACTUAL`

`RELEASE_STATE_ACTUAL` compares predicted and actual forward speed, vertical speed, and rel_alt when R is reached.

## Validation performed here

- Python syntax compilation passed for `uav_fcu_interface` and `uav_bringup` Python sources.
- Non-ROS unit suite: **133 passed**.

No SITL or ROS/MAVROS runtime was available in this container, so a real SITL run still needs to be performed on the project machine.

## First recommended runtime test

Run SITL first and inspect this ordering:

`B_CROSSED -> AB_PREDICTION_INPUT -> R_PREDICTION_ITERATION -> R_CALC_DONE -> SECOND_FULL_PUSH -> SECOND_FULL_PULL -> SECOND_FULL_VERIFY_PASS -> DYNAMIC_MISSION_VERIFIED -> U_REACHED -> R_REACHED -> RELEASE_STATE_ACTUAL`

Pay special attention to:

- predicted RC stability between runs
- vertical fit mode (`trend` vs `weighted_mean`)
- predicted vs actual `vz_R`
- predicted vs actual relative altitude at R
- `DYNAMIC_MISSION_VERIFIED` still occurring before U reached
- dynamic RC staying inside U/R geometry constraints

## Not modeled yet

- lateral R movement
- wind
- aerodynamic drag of the payload
- target-vs-Home terrain elevation difference
- non-zero release mechanism delay
- R acceptance-radius trigger bias
