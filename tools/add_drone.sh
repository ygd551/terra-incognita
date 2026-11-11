#!/bin/bash
# The RMF-Obelix multirotor for sim/drone_sim.py (./sim.sh --robot rmf_obelix_ouster64|rmf_obelix_avia): downloads its mesh from
# ntnu-arl/rotors_simulator (branch dev/rmf_obelix, pinned commit; package rotors_description, license "ASL 2.0" =
# Apache-2.0 in its package.xml) into assets/drones/rmf_obelix/ (not redistributed by this repository) with an
# ATTRIBUTION.md, and builds assets/drones/rmf_obelix/rmf_obelix.usd (sim/drone_to_usd.py: mass, inertia and frame
# from config/robots.yaml).
#   tools/add_drone.sh
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
D=$ROOT/assets/drones/rmf_obelix
COMMIT=e8c6f170de51620f572664656f10603334bf5057
URL=https://raw.githubusercontent.com/ntnu-arl/rotors_simulator/$COMMIT/rotors_description/meshes/rmf_obelix
mkdir -p "$D"
for f in rmf_obelix.dae carbon_fiber.jpg; do
  [ -s "$D/$f" ] || curl -fsSL --retry 5 -o "$D/$f" "$URL/$f"
done
cat > "$D/ATTRIBUTION.md" <<EOF
# RMF-Obelix model

\`rmf_obelix.dae\` and \`carbon_fiber.jpg\` are downloaded unchanged from
https://github.com/ntnu-arl/rotors_simulator/tree/$COMMIT/rotors_description/meshes/rmf_obelix
(NTNU Autonomous Robots Lab's fork of RotorS by ETH Zurich ASL). License: Apache-2.0 ("ASL 2.0" in
rotors_description/package.xml; the repository has no separate license file). \`rmf_obelix.usd\` and \`mesh/\` are
converted from them by sim/drone_to_usd.py.

RotorS: F. Furrer, M. Burri, M. Achtelik, R. Siegwart, "RotorS - A Modular Gazebo MAV Simulator Framework", in Robot
Operating System (ROS): The Complete Reference (Volume 1), Springer, 2016, doi:10.1007/978-3-319-26054-9_23.
EOF
"$ROOT/container.sh" start
docker exec isaac_explore bash -c "cd /workspace/isaac_explore && /isaac-sim/python.sh sim/drone_to_usd.py" 2>&1 \
  | grep -E "drone_to_usd|Traceback|Error:"
docker exec isaac_explore chown -R "$(id -u):$(id -g)" /isaac_explore/assets/drones
