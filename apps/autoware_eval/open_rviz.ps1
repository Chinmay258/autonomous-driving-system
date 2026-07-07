# Open (or reopen) rviz for the running autoware_viz sim.
# Runs rviz as a standalone process INSIDE the sim container; closing the
# window just ends this process — the headless sim keeps running and nothing
# respawns rviz. Run again to reopen.  Usage: powershell -File apps/autoware_eval/open_rviz.ps1
$ErrorActionPreference = "Stop"
$cfg = "/opt/autoware/share/autoware_launch/rviz/autoware.rviz"
Write-Host "opening rviz (close its window any time; it will not reopen on its own)..."
docker exec -d autoware_viz bash -lc "source /opt/autoware/setup.bash 2>/dev/null; rviz2 -d $cfg >/tmp/rviz.log 2>&1"
Write-Host "rviz launched. Reopen later with: powershell -File apps/autoware_eval/open_rviz.ps1"
