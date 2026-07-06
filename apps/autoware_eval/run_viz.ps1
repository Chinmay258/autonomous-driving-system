# Visual Autoware planning simulator on the Barcelona map (rviz via WSLg).
# Usage: powershell -File apps/autoware_eval/run_viz.ps1  (from repo root)
# The rviz window appears on the Windows desktop. Teardown:
#   docker rm -f autoware_viz
$ErrorActionPreference = "Stop"
$root = (Resolve-Path "$PSScriptRoot\..\..").Path
$image = "ghcr.io/autowarefoundation/autoware:latest-runtime"

docker rm -f autoware_viz 2>$null | Out-Null
# 12 cpus: rviz renders in software (llvmpipe) and needs ~3 cores; with only 6
# the control loop starves, flunks its rate monitors and latches a stop.
# The pid.param.yaml override disables keep-stopped-until-steer-convergence,
# which otherwise deadlocks at standstill in this setup.
docker run -d --name autoware_viz --memory=4g --memory-swap=4g --cpus=12 `
  -e DISPLAY=:0 -e WAYLAND_DISPLAY=wayland-0 `
  -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e PULSE_SERVER=/mnt/wslg/PulseServer `
  -e LIBGL_ALWAYS_SOFTWARE=1 `
  -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix `
  -v /run/desktop/mnt/host/wslg:/mnt/wslg `
  -v "${root}\apps\autoware_eval\mapdir:/autoware_map:ro" `
  -v "${root}\apps\autoware_eval:/eval:ro" `
  -v "${root}\apps\autoware_eval\overrides\pid.param.yaml:/opt/autoware/share/autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml:ro" `
  $image `
  ros2 launch autoware_launch planning_simulator.launch.xml `
  map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit | Out-Null

Write-Host "waiting for the stack (map + planner)..."
do { Start-Sleep 5 } until ((docker logs autoware_viz 2>&1 | Select-String "waiting odometry"))

Write-Host "placing vehicle + goal (cross-map scenario)..."
docker exec autoware_viz bash -lc "source /opt/autoware/setup.bash 2>/dev/null; python3 /eval/golden_route_check.py --map /autoware_map/lanelet2_map.osm --origin-lat 41.391464400000004 --origin-lon 2.16537545 --start-id 1 --goal-id 1855 --timeout 120" | Select-String "GOLDEN_ROUTE=" | Out-Null

Write-Host "engaging autonomous mode (retries while diagnostics settle)..."
foreach ($i in 1..6) {
  $ok = docker exec autoware_viz bash -lc "source /opt/autoware/setup.bash 2>/dev/null; timeout 25 ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode {} 2>&1" | Select-String "success=True"
  if ($ok) { Write-Host "engaged - the car is driving in rviz."; break }
  Start-Sleep 8
}
Write-Host "manual controls in rviz: '2D Pose Estimate' to place the car, '2D Goal Pose' to set a destination, OperationMode panel AUTO to engage."