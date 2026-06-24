# Real Bird Detector Contract

The real detector is a conservative classifier wrapper. It does not publish
navigation goals, motor commands, sound commands, or deterrence events.

## Inputs

- `/camera/image_raw`: `sensor_msgs/msg/Image`
- `/camera/camera_info`: `sensor_msgs/msg/CameraInfo`
- `/waver/camera_target_centered`: `std_msgs/msg/Bool`
- Optional model path parameter: `model_path` or `bird_model_path`

## Outputs

- `/waver/bird_detections_2d`: `vision_msgs/msg/Detection2DArray`
- `/waver/bird_confirmed`: `std_msgs/msg/Bool`
- `/waver/bird_detector_state`: `std_msgs/msg/String`
- `/waver/target_class`: `std_msgs/msg/String`
- `/waver/target_confidence`: `std_msgs/msg/Float32`
- `/waver/target_classification_state`: `std_msgs/msg/String`
- `/waver/target_classification_latency_ms`: `std_msgs/msg/Float32`

## Parameters

- `backend` / `bird_backend`: currently `yolo` for real inference.
- `model_path` / `bird_model_path`: required in real profile.
- `confidence_threshold`: default `0.65`.
- `accepted_bird_classes`: default `["bird"]`.
- `non_bird_classes`: default `["person", "vehicle", "robot", "drone", "irrelevant", "none"]`.
- `unknown_confidence_threshold`: default `0.50`.
- `nof_m_window`, `nof_m_required`: temporal voting window.
- `max_detector_latency_sec`: contract field for monitoring.
- `detector_required_for_real`: if true, missing model is a hard unavailable state.

## Class Normalization

The pure helper `waver_patrol.perception.bird_classification` normalizes class
names before decision logic. Examples:

- `bird`, `Bird`, `BIRD` -> `bird`
- `car`, `truck` -> `vehicle`
- `uav` -> `drone`
- `airplane` -> `irrelevant`

Only normalized `bird` above threshold can set `bird_confirmed=true`.

## Failure Behavior

- Empty model path: `MODEL_MISSING`, `bird_confirmed=false`.
- Missing model file: `MODEL_NOT_FOUND`, `bird_confirmed=false`.
- Unsupported backend: `BACKEND_UNSUPPORTED`, `bird_confirmed=false`.
- Camera stale: `CAMERA_STALE`, `bird_confirmed=false`.
- Camera not aligned when alignment is required: `WAIT_CAMERA_ALIGNMENT`, `bird_confirmed=false`.
- Inference error: `INFERENCE_ERROR`, `bird_confirmed=false`.

Camera-only classification is not enough for navigation. Real target approach
also requires 3D LiDAR / PointCloud / fusion validity and mission safety gates.
Sound/deterrent output remains blocked unless bird confirmation, target lock,
and explicit sound safety acknowledgement are all true.

## Claims Boundary

This repository does not include a validated real bird dataset or model
accuracy report. Do not claim mAP, precision, recall, or outdoor bird
deterrence performance from Gazebo-only runs.
