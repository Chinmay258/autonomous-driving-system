# End-to-end Autoware golden evaluation (headless).
# Usage: powershell -File apps/autoware_eval/run_eval.ps1  (from repo root)
$ErrorActionPreference = "Stop"
$image = "ghcr.io/autowarefoundation/autoware:latest-runtime"
$root = (Resolve-Path "$PSScriptRoot\..\..").Path

python -m uv run python apps/autoware_eval/prep_map.py

$meta = python -m uv run python apps/autoware_eval/compare_routes.py scenarios | ConvertFrom-Json
$lat = $meta.origin.lat; $lon = $meta.origin.lng

docker rm -f autoware_eval 2>$null
docker run -d --name autoware_eval --memory=4g --memory-swap=4g --cpus=6 `
  -v "${root}\apps\autoware_eval\mapdir:/autoware_map:ro" `
  -v "${root}\apps\autoware_eval:/eval:ro" `
  $image `
  ros2 launch autoware_launch planning_simulator.launch.xml `
  map_path:=/autoware_map vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit rviz:=false

Write-Host "warming up planning simulator (60 s)..."
Start-Sleep 60

foreach ($s in $meta.scenarios) {
  Write-Host "=== scenario $($s.name): $($s.start) -> $($s.goal) ==="
  $golden = docker exec autoware_eval bash -lc "source /opt/autoware/setup.bash 2>/dev/null || source /autoware/install/setup.bash; python3 /eval/golden_route_check.py --map /autoware_map/lanelet2_map.osm --origin-lat $lat --origin-lon $lon --start-id $($s.start) --goal-id $($s.goal)" |
    Select-String "^GOLDEN_ROUTE=" | ForEach-Object { $_.Line.Substring(13) }
  if (-not $golden -or $golden -eq "TIMEOUT") { Write-Host "  golden probe failed"; continue }
  python -m uv run python apps/autoware_eval/compare_routes.py compare `
    --start-id $s.start --goal-id $s.goal --golden $golden
}

docker rm -f autoware_eval | Out-Null
Write-Host "done."
