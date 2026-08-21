#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

set +u
source "${ROOT_DIR}/install/setup.bash"
set -u

WORLD_TEMPLATE="${ROOT_DIR}/src/robot_simulation/worlds/indoor_room.sdf.xacro"
GEOMETRY_FILE="${ROOT_DIR}/src/robot_description/config/robot_geometry.yaml"
SIMULATION_PHYSICS_FILE="${ROOT_DIR}/src/robot_simulation/config/simulation_physics.yaml"
SIMULATION_MEDIA_DIRECTORY="${ROOT_DIR}/src/robot_simulation/media"
RVIZ_CONFIG="${ROOT_DIR}/src/robot_simulation/rviz/amr_sim.rviz"
TEXT_MESH_DIR="${ROOT_DIR}/src/robot_simulation/media/text_meshes"

required_files=(
  "${WORLD_TEMPLATE}"
  "${GEOMETRY_FILE}"
  "${SIMULATION_PHYSICS_FILE}"
  "${RVIZ_CONFIG}"
  "${TEXT_MESH_DIR}/label_robot_1.obj"
  "${TEXT_MESH_DIR}/label_obstacle_clear_status.obj"
  "${TEXT_MESH_DIR}/label_text_black.mtl"
  "${ROOT_DIR}/src/robot_bringup/launch/amr_demo.launch.py"
  "${ROOT_DIR}/src/robot_simulation/launch/sim.launch.py"
  "${ROOT_DIR}/scripts/generate_amr_sim_text_geometry_labels.py"
  "${ROOT_DIR}/scripts/check_full_amr_simulation_stack.sh"
  "${ROOT_DIR}/scripts/check_amr_task_dashboard_showcase.sh"
  "${ROOT_DIR}/scripts/check_amr_rviz_showcase_topics.sh"
  "${ROOT_DIR}/scripts/check_amr_sim_gazebo_entities.sh"
  "${ROOT_DIR}/scripts/check_amr_sim_two_robot_traffic_wait.sh"
  "${ROOT_DIR}/scripts/check_amr_facility_sequence_showcase.sh"
  "${ROOT_DIR}/scripts/check_docking_retry.sh"
  "${ROOT_DIR}/scripts/check_dynamic_speed_limit.sh"
  "${ROOT_DIR}/scripts/check_amr_obstacle_safety_stop.sh"
)

for file in "${required_files[@]}"; do
  if [[ ! -f "${file}" ]]; then
    echo "required showcase asset missing: ${file}" >&2
    exit 1
  fi
done

required_scripts=(
  "${ROOT_DIR}/scripts/check_full_amr_simulation_stack.sh"
  "${ROOT_DIR}/scripts/check_amr_task_dashboard_showcase.sh"
  "${ROOT_DIR}/scripts/check_amr_rviz_showcase_topics.sh"
  "${ROOT_DIR}/scripts/check_amr_sim_gazebo_entities.sh"
  "${ROOT_DIR}/scripts/check_amr_sim_two_robot_traffic_wait.sh"
  "${ROOT_DIR}/scripts/check_amr_facility_sequence_showcase.sh"
  "${ROOT_DIR}/scripts/check_docking_retry.sh"
  "${ROOT_DIR}/scripts/check_dynamic_speed_limit.sh"
  "${ROOT_DIR}/scripts/check_amr_obstacle_safety_stop.sh"
)

for script in "${required_scripts[@]}"; do
  if [[ ! -x "${script}" ]]; then
    echo "showcase script is not executable: ${script}" >&2
    exit 1
  fi
done

rg "任务看板|mission_panel|LaserScan|Map|Path|Goal" "${RVIZ_CONFIG}"
rg "showcase_environment_polish|label_robot_1|label_obstacle_clear_status" "${WORLD_TEMPLATE}"
rg -F 'file://${simulation_media_path}/text_meshes/label_robot_1.obj' "${WORLD_TEMPLATE}" >/dev/null
rg -F 'file://${simulation_media_path}/text_meshes/label_obstacle_clear_status.obj' "${WORLD_TEMPLATE}" >/dev/null
rg "<ambient>0.000 0.000 0.000 1</ambient>|<diffuse>0.000 0.000 0.000 1</diffuse>" "${WORLD_TEMPLATE}" >/dev/null
rg "mtllib label_text_black.mtl|usemtl label_text_black" "${TEXT_MESH_DIR}/label_robot_1.obj" "${TEXT_MESH_DIR}/label_door.obj" >/dev/null
rg "Kd 0.000 0.000 0.000|Ka 0.000 0.000 0.000" "${TEXT_MESH_DIR}/label_text_black.mtl" >/dev/null
python3 - "${TEXT_MESH_DIR}" <<'PY'
import sys
from pathlib import Path

mesh_dir = Path(sys.argv[1])
bad = []
for path in sorted(mesh_dir.glob("label_*.obj")):
    xs = []
    ys = []
    has_black_material = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if line == "usemtl label_text_black":
            has_black_material = True
        if not line.startswith("v "):
            continue
        _, x, y, _z = line.split()
        xs.append(float(x))
        ys.append(float(y))
    if not xs or not ys:
        bad.append(f"{path.name}: no mesh vertices")
        continue
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
    if not has_black_material:
        bad.append(f"{path.name}: missing black material")
    if height <= width * 1.25:
        bad.append(f"{path.name}: not vertical enough, width={width:.3f}, height={height:.3f}")

if bad:
    print("Gazebo text labels must stay black vertical OBJ meshes:", file=sys.stderr)
    print("\n".join(bad), file=sys.stderr)
    sys.exit(1)
PY
if rg "albedo_map|../media/labels|<plane><normal>0 0 1</normal><size>" "${WORLD_TEMPLATE}" >/dev/null; then
  echo "Gazebo labels must be texture-free geometry, not planes or PNG albedo maps" >&2
  exit 1
fi
if [[ "$(find "${TEXT_MESH_DIR}" -maxdepth 1 -name 'label_*.obj' | wc -l)" -lt 28 ]]; then
  echo "Gazebo text label mesh assets are incomplete" >&2
  exit 1
fi

python3 -m py_compile \
  "${ROOT_DIR}/src/robot_bringup/launch/amr_demo.launch.py" \
  "${ROOT_DIR}/src/robot_simulation/launch/sim.launch.py" \
  "${ROOT_DIR}/scripts/generate_amr_sim_text_geometry_labels.py"

for script in "${required_scripts[@]}"; do
  bash -n "${script}"
done
GENERATED_WORLD="$(mktemp /tmp/robot_showcase_world.XXXXXX.sdf)"
trap 'rm -f "${GENERATED_WORLD}"' EXIT
xacro "${WORLD_TEMPLATE}" \
  geometry_file:="${GEOMETRY_FILE}" \
  simulation_physics_file:="${SIMULATION_PHYSICS_FILE}" \
  simulation_media_directory:="${SIMULATION_MEDIA_DIRECTORY}" \
  -o "${GENERATED_WORLD}"
gz sdf -k "${GENERATED_WORLD}" >/dev/null
ros2 launch robot_bringup amr_demo.launch.py --show-args >/dev/null

echo "AMR showcase demo assets passed"
