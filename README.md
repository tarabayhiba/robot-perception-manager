# robot-perception-manager

ROS 2 package integrating pub/sub, services, actions, parameters,
and TF2 into a single perception pipeline: an action server that streams
simulated object detections, a service to change the confidence threshold
at runtime, and a static TF broadcaster for the camera frame.

## Architecture

```
  all nodes launched under namespace /perception (perception_pipeline.launch.py)

   camera_tf_broadcaster                     perception_manager
   ----------------------                   ------------------
   params from YAML:                         param: confidence_threshold (YAML,
     tx ty tz roll pitch yaw                    live-updatable via service)
     
   publishes static tf                       action server /start_detection
   base_link -> camera_optical_frame          - preemptable: new goal cancels
   (one-shot, /tf_static)                       the running one, starts fresh
                                              service server /set_confidence
                                                - validates threshold in [0,1]
                                              publisher /detections (~10Hz)
                                                - only while a goal is active
                                                        | goal / feedback /
                                                        | cancel / result
                                                        v
                                              perception_client
                                              - param: target_class
                                              - sends goal, logs feedback
                                                (fps, detections_so_far)
                                              - Ctrl+C -> cancels goal
```

`set_confidence` can be called at any time (including mid-goal) from a
separate terminal via `ros2 service call`, and takes effect on the next
published detection since `perception_manager` re-reads the parameter
every loop tick rather than caching it at goal start.

## Build

```bash
cd ros2_ws
colcon build --packages-select my_interfaces perception_pipeline
source install/setup.bash
```

## Run

Launch the whole system (manager, TF broadcaster, and a client that
sends a goal for `target_class` from YAML):

```bash
ros2 launch perception_pipeline perception_pipeline.launch.py
```

Or run nodes individually, in separate terminals (
`source install/setup.bash` first):

```bash
ros2 run perception_pipeline perception_manager
ros2 run perception_pipeline camera_tf_broadcaster
ros2 run perception_pipeline perception_client
```

Send a goal manually and watch feedback:

```bash
ros2 action send_goal /perception/start_detection \
  my_interfaces/action/StartDetection "{target_class: 'cup'}" --feedback
```

Change the confidence threshold mid-run:

```bash
ros2 service call /perception/set_confidence \
  my_interfaces/srv/SetConfidenceThreshold "{threshold: 0.9}"
```

Cancel a running goal: `Ctrl+C` on the `ros2 action send_goal` process
(or on `perception_client` — it catches SIGINT and forwards it as a
cancel request instead of just dying).

Verify the TF tree:

```bash
ros2 run tf2_tools view_frames
```


# perception_manager manual test log (alone)

Build and run commands used (from `ros2_ws/`):

```bash
colcon build --packages-select my_interfaces perception_pipeline
source install/setup.bash
ros2 run perception_pipeline perception_manager
```

`target_class` is specified as a field in the **goal** you send not a
separate flag it's part of the goal message:

```bash
ros2 action send_goal /start_detection my_interfaces/action/StartDetection \
  "{target_class: 'cup'}" --feedback
#                ^^^^^ this is where you set change 'cup' for anything
```

## 1. Basic run + feedback 

```bash
ros2 action send_goal /start_detection my_interfaces/action/StartDetection \
  "{target_class: 'cup'}" --feedback
```

Feedback observed, `detections_so_far` incrementing by 2 each message
(published every 2nd tick of the 10Hz loop):


Measured with `ros2 topic hz` on the feedback topic while
a goal was running:

```bash
ros2 topic hz /start_detection/_action/feedback
```

```
average rate: 5.001
	min: 0.200s max: 0.200s std dev: 0.00012s window: 6
average rate: 5.000
	min: 0.200s max: 0.200s std dev: 0.00013s window: 11
```

5 Hz = one feedback message every **0.200s**, exactly as required (min and
max interval both pinned at 0.200s, near-zero std dev).

## 2. Client-initiated cancel

```bash
ros2 action send_goal /start_detection my_interfaces/action/StartDetection \
  "{target_class: 'cup'}" --feedback
# Ctrl+C (SIGINT) partway through
```


Node stayed alive (`ros2 node list` still showed `/perception_manager`
afterward).

## 3. BONUS: Preemption (a second goal supersedes a running one) 

```bash
# terminal A
ros2 action send_goal /start_detection my_interfaces/action/StartDetection \
  "{target_class: 'cup'}" --feedback
# ~2s later terminal B, while A is still running:
ros2 action send_goal /start_detection my_interfaces/action/StartDetection \
  "{target_class: 'bottle'}" --feedback
```



**Bug found and fixed during this test:** the first implementation called
`goal_handle->canceled(result)` for the superseded goal. That's only a valid
state transition from `CANCELING` a superseded goal was never asked to
cancel by its own client, so its underlying state is still `EXECUTING`, &
calling `canceled()` from `EXECUTING` throws
`rclcpp::exceptions::RCLError: invalid transition from state EXECUTING with
event CANCELED` and **crashes the whole node**. Fixed by calling
`goal_handle->abort(result)` instead (a valid `EXECUTING -> ABORTED`
transition)

## 4. set_confidence service

Out-of-range rejection:
```bash
ros2 service call /set_confidence my_interfaces/srv/SetConfidenceThreshold \
  "{threshold: 1.5}"
```


Valid update:
```bash
ros2 service call /set_confidence my_interfaces/srv/SetConfidenceThreshold \
  "{threshold: 0.9}"
```

