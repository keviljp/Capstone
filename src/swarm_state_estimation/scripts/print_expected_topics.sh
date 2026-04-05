#!/usr/bin/env bash
cat <<'EOF'
Expected topics:
  /camera/image_raw
  /camera/image_rect
  /camera/camera_info
  /detections
  /swarm_state/discovered_robot_id
  /swarmbot_<id>/tag_pose
  /swarmbot_<id>/imu/data
  /swarmbot_<id>/odom
EOF
