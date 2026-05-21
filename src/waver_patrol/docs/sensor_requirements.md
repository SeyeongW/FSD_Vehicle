# Sensor Requirements

Minimum supervised patrol requires stable LiDAR `/scan`, stable `/tf`, usable local pose, and a tested
serial stop path. Camera is optional for LiDAR-only safety patrol, but without camera object detection
Waver cannot classify humans, vehicles, bikes, or animals. Without encoders, slip and actual speed
estimation are approximate. Without GPS/RTK, geographic outdoor patrol is disabled.
