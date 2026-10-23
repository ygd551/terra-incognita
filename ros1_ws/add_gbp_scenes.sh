#!/bin/bash
# The sample worlds of ntnu-arl/gbplanner_ros as simulation scenes (gbp_*): real mine scans edgar_mine,
# pittsburgh_mine, virginia_mine (meshes in the repository), the tile caves darpa_cave_01/02 (gbplanner2) and
# urban_cave_transition (gbplanner3, tiles from Gazebo Fuel). Writes assets/subt/worlds/gbp_*.sdf and the mine models
# into the Fuel cache layout, then converts them with tools/add_scenes.sh. Start poses: config/scenes.yaml.
#   ros1_ws/add_gbp_scenes.sh        (uses the gbplanner_ros checkout of ./fetch_src.sh, fetching it if needed)
set -e
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/.." && pwd)
SIM=$HERE/src/gbplanner_ros/planner_gazebo_sim
[ -d "$SIM" ] || "$HERE/fetch_src.sh"
W=$ROOT/assets/subt/worlds; mkdir -p "$W"
FUEL=https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models
GBP3=https://raw.githubusercontent.com/ntnu-arl/gbplanner_ros/f9904bb69392d2d46b2cbe939762f967a65daa14   # gbplanner3

# mine scans: model -> the Fuel cache layout (sim/sdf_to_usd.py resolves model://NAME there), world = the model at the
# pose of its model.sdf (an <include> without a pose takes it in Gazebo; sdf_to_usd.py uses the include's pose only)
for m in edgar_mine pittsburgh_mine virginia_mine; do
  mkdir -p "$ROOT/assets/fuel/local/gbplanner/models/$m/1" && cp "$SIM/models/$m/"* "$ROOT/assets/fuel/local/gbplanner/models/$m/1/"
  pose=$(sed -n 's|.*<pose>\(.*\)</pose>.*|\1|p' "$SIM/models/$m/model.sdf" | head -1)
  cat > "$W/gbp_$m.sdf" <<EOF
<?xml version="1.0" ?>
<!-- isaac_explore: $m from ntnu-arl/gbplanner_ros (gbplanner2) planner_gazebo_sim, the model at its model.sdf pose -->
<sdf version="1.6">
  <world name="gbp_$m">
    <include>
      <name>$m</name>
      <pose>$pose</pose>
      <uri>model://$m</uri>
    </include>
  </world>
</sdf>
EOF
done
# tile worlds: their model:// tiles are Gazebo Fuel models (OpenRobotics)
for w in darpa_cave_01 darpa_cave_02; do
  sed -E "s#<uri>model://([^<]*)</uri>#<uri>$FUEL/\1</uri>#" "$SIM/worlds/$w.world" |
    sed "2i <!-- isaac_explore: $w from ntnu-arl/gbplanner_ros (gbplanner2), model:// tiles pointed at Fuel -->" > "$W/gbp_$w.sdf"
done
curl -fsSL "$GBP3/planner_gazebo_sim/worlds/urban_cave_transition.world" |
  sed -E "s#<uri>model://(Urban Cave Transition)</uri>#<uri>$FUEL/\1</uri>#" |
  sed "2i <!-- isaac_explore: urban_cave_transition from ntnu-arl/gbplanner_ros (gbplanner3), the tile pointed at Fuel -->" \
  > "$W/gbp_urban_cave_transition.sdf"

"$ROOT/tools/add_scenes.sh" gbp_edgar_mine gbp_pittsburgh_mine gbp_virginia_mine gbp_darpa_cave_01 gbp_darpa_cave_02 \
  gbp_urban_cave_transition
