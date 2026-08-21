import copy
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml


def _source_directory(variable_name):
    entries = [
        os.path.realpath(entry)
        for entry in os.environ.get(variable_name, "").split(os.pathsep)
        if entry
    ]
    source_directories = sorted(
        {entry for entry in entries if os.path.isdir(entry)}
    )
    if len(source_directories) != 1:
        raise RuntimeError(
            f"{variable_name} must contain exactly one existing source directory; "
            f"resolved candidates: {source_directories}"
        )
    return source_directories[0]


DESCRIPTION_DIR = _source_directory("ROBOT_DESCRIPTION_SOURCE_DIR")
SIMULATION_DIR = _source_directory("ROBOT_SIMULATION_SOURCE_DIR")
NAVIGATION_DIR = _source_directory("ROBOT_NAVIGATION_SOURCE_DIR")
HARDWARE_DIR = _source_directory("ROBOT_HARDWARE_SOURCE_DIR")
BRINGUP_DIR = _source_directory("ROBOT_BRINGUP_SOURCE_DIR")

GEOMETRY_FILE = os.path.join(DESCRIPTION_DIR, "config", "robot_geometry.yaml")
URDF_FILE = os.path.join(DESCRIPTION_DIR, "urdf", "robot.urdf.xacro")
PHYSICS_FILE = os.path.join(SIMULATION_DIR, "config", "simulation_physics.yaml")
SDF_FILE = os.path.join(SIMULATION_DIR, "worlds", "indoor_room.sdf.xacro")
MEDIA_DIRECTORY = os.path.join(SIMULATION_DIR, "media")
FOOTPRINT_FILE = os.path.join(NAVIGATION_DIR, "config", "robot_footprint.yaml")
CALIBRATION_FILE = os.path.join(HARDWARE_DIR, "config", "chassis_calibration.yaml")
GAZEBO_CONTROLLERS_FILE = os.path.join(BRINGUP_DIR, "config", "controllers.yaml")


def _load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _expand(path, **mappings):
    document = xacro.process_file(path, mappings=mappings)
    return ET.fromstring(document.toxml())


def _numbers(text):
    return [float(value) for value in text.split()]


def _assert_vector(actual, expected, tolerance=1e-9):
    assert len(actual) == len(expected)
    for actual_value, expected_value in zip(actual, expected):
        assert math.isclose(actual_value, expected_value, abs_tol=tolerance)


def _urdf_joint(root, name):
    joint = root.find(f"./joint[@name='{name}']")
    assert joint is not None
    return joint


def _sdf_model(root, name):
    model = root.find(f"./world/model[@name='{name}']")
    assert model is not None
    return model


def test_reference_geometry_expands_to_expected_urdf_frames():
    geometry = _load_yaml(GEOMETRY_FILE)
    root = _expand(URDF_FILE, geometry_file=GEOMETRY_FILE)

    assert math.isclose(
        geometry["frames"]["base_link_height_m"],
        geometry["drive_wheels"]["radius_m"],
        abs_tol=1e-9,
    )
    assert math.isclose(geometry["drive_wheels"]["joint_z_m"], 0.0, abs_tol=1e-9)

    base_joint = _urdf_joint(root, "base_footprint_joint")
    _assert_vector(
        _numbers(base_joint.find("origin").attrib["xyz"]),
        [0.0, 0.0, geometry["frames"]["base_link_height_m"]],
    )
    for name, side in [("left_wheel_joint", 1.0), ("right_wheel_joint", -1.0)]:
        origin = _numbers(_urdf_joint(root, name).find("origin").attrib["xyz"])
        _assert_vector(
            origin,
            [
                geometry["drive_wheels"]["joint_x_m"],
                side * geometry["drive_wheels"]["center_separation_m"] / 2.0,
                geometry["drive_wheels"]["joint_z_m"],
            ],
        )

    caster_origin = _numbers(
        _urdf_joint(root, "caster_wheel_joint").find("origin").attrib["xyz"]
    )
    _assert_vector(caster_origin, geometry["caster"]["xyz_m"])
    for sensor_name in ["lidar", "imu", "camera"]:
        sensor = geometry["sensors"][sensor_name]
        joint = _urdf_joint(root, f"{sensor_name}_joint")
        assert joint.find("parent").attrib["link"] == sensor["parent_frame"]
        assert joint.find("child").attrib["link"] == sensor["frame_id"]
        _assert_vector(_numbers(joint.find("origin").attrib["xyz"]), sensor["xyz_m"])
        _assert_vector(_numbers(joint.find("origin").attrib["rpy"]), sensor["rpy_rad"])


def test_sdf_instances_share_geometry_and_keep_topics_isolated(tmp_path):
    geometry = _load_yaml(GEOMETRY_FILE)
    simulation = _load_yaml(PHYSICS_FILE)
    root = _expand(
        SDF_FILE,
        geometry_file=GEOMETRY_FILE,
        simulation_physics_file=PHYSICS_FILE,
        simulation_media_directory=MEDIA_DIRECTORY,
    )
    expected_topics = {
        "mobile_robot": ("/cmd_vel", "/sim/scan", "/sim/imu"),
        "mobile_robot_2": (
            "/robot_2/cmd_vel",
            "/robot_2/sim/scan",
            "/robot_2/sim/imu",
        ),
    }
    for model_name, topics in expected_topics.items():
        model = _sdf_model(root, model_name)
        assert math.isclose(
            _numbers(model.find("pose").text)[2],
            geometry["frames"]["base_link_height_m"],
            abs_tol=1e-9,
        )
        base = model.find("./link[@name='base_link']")
        assert base is not None
        _assert_vector(
            _numbers(base.find("./collision[@name='base_collision']/geometry/box/size").text),
            [
                geometry["base"]["length_m"],
                geometry["base"]["width_m"],
                geometry["base"]["height_m"],
            ],
        )
        assert math.isclose(
            float(base.find("./inertial/mass").text),
            geometry["base"]["mass_kg"],
            abs_tol=1e-9,
        )
        for wheel_name, side in [("left_wheel", 1.0), ("right_wheel", -1.0)]:
            wheel = model.find(f"./link[@name='{wheel_name}']")
            assert wheel is not None
            pose = _numbers(wheel.find("pose").text)
            assert math.isclose(
                pose[1],
                side * geometry["drive_wheels"]["center_separation_m"] / 2.0,
                abs_tol=1e-9,
            )
            assert math.isclose(
                float(wheel.find("./collision/geometry/cylinder/radius").text),
                geometry["drive_wheels"]["radius_m"],
                abs_tol=1e-9,
            )
        plugin = model.find("./plugin[@name='gz::sim::systems::DiffDrive']")
        assert plugin is not None
        assert math.isclose(
            float(plugin.find("wheel_separation").text),
            geometry["drive_wheels"]["center_separation_m"],
            abs_tol=1e-9,
        )
        assert math.isclose(
            float(plugin.find("wheel_radius").text),
            geometry["drive_wheels"]["radius_m"],
            abs_tol=1e-9,
        )
        assert plugin.find("topic").text == topics[0]
        lidar_sensor = base.find("./sensor[@name='lidar']")
        imu_sensor = base.find("./sensor[@name='imu']")
        assert lidar_sensor is not None
        assert imu_sensor is not None
        assert lidar_sensor.find("topic").text == topics[1]
        assert imu_sensor.find("topic").text == topics[2]
        _assert_vector(
            _numbers(lidar_sensor.find("pose").text),
            geometry["sensors"]["lidar"]["xyz_m"]
            + geometry["sensors"]["lidar"]["rpy_rad"],
        )
        _assert_vector(
            _numbers(imu_sensor.find("pose").text),
            geometry["sensors"]["imu"]["xyz_m"]
            + geometry["sensors"]["imu"]["rpy_rad"],
        )
        _assert_vector(
            _numbers(base.find("./visual[@name='camera_visual']/pose").text),
            geometry["sensors"]["camera"]["xyz_m"]
            + geometry["sensors"]["camera"]["rpy_rad"],
        )
        _assert_vector(
            _numbers(base.find("./collision[@name='camera_collision']/pose").text),
            geometry["sensors"]["camera"]["xyz_m"]
            + geometry["sensors"]["camera"]["rpy_rad"],
        )
        assert math.isclose(
            float(lidar_sensor.find("update_rate").text),
            simulation["sensors"]["lidar"]["update_rate_hz"],
            abs_tol=1e-9,
        )
        assert math.isclose(
            float(imu_sensor.find("update_rate").text),
            simulation["sensors"]["imu"]["update_rate_hz"],
            abs_tol=1e-9,
        )
        assert base.find("./sensor[@name='camera']") is None

    for uri in root.findall(".//mesh/uri"):
        assert uri.text.startswith(f"file://{MEDIA_DIRECTORY}/")
        assert os.path.isfile(uri.text.removeprefix("file://"))

    generated = tmp_path / "indoor_room.sdf"
    generated.write_text(ET.tostring(root, encoding="unicode"), encoding="utf-8")
    validation = subprocess.run(
        ["gz", "sdf", "-k", str(generated)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert validation.returncode == 0, validation.stderr or validation.stdout


def test_use_gazebo_sensor_parameters_come_from_simulation_physics():
    simulation = _load_yaml(PHYSICS_FILE)
    root = _expand(
        URDF_FILE,
        geometry_file=GEOMETRY_FILE,
        simulation_physics_file=PHYSICS_FILE,
        gazebo_controllers_file=GAZEBO_CONTROLLERS_FILE,
        use_gazebo="true",
    )

    lidar = root.find("./gazebo[@reference='lidar_link']/sensor[@name='lidar']")
    imu = root.find("./gazebo[@reference='imu_link']/sensor[@name='imu']")
    assert lidar is not None
    assert imu is not None
    lidar_config = simulation["sensors"]["lidar"]
    assert math.isclose(
        float(lidar.find("update_rate").text),
        lidar_config["update_rate_hz"],
        abs_tol=1e-9,
    )
    assert int(lidar.find("./ray/scan/horizontal/samples").text) == lidar_config["samples"]
    assert math.isclose(
        float(lidar.find("./ray/scan/horizontal/resolution").text),
        lidar_config["angular_resolution"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(lidar.find("./ray/scan/horizontal/min_angle").text),
        lidar_config["min_angle_rad"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(lidar.find("./ray/scan/horizontal/max_angle").text),
        lidar_config["max_angle_rad"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(lidar.find("./ray/range/min").text),
        lidar_config["min_range_m"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(lidar.find("./ray/range/max").text),
        lidar_config["max_range_m"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(lidar.find("./ray/range/resolution").text),
        lidar_config["range_resolution_m"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(lidar.find("./ray/noise/stddev").text),
        lidar_config["noise_stddev_m"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(imu.find("update_rate").text),
        simulation["sensors"]["imu"]["update_rate_hz"],
        abs_tol=1e-9,
    )
    assert root.find("./gazebo[@reference='camera_link']/sensor[@name='camera']") is None


def test_camera_enabled_controls_urdf_and_sdf_sensor_generation(tmp_path):
    geometry = _load_yaml(GEOMETRY_FILE)
    simulation = copy.deepcopy(_load_yaml(PHYSICS_FILE))
    simulation["sensors"]["camera"]["enabled"] = True
    enabled_physics_file = tmp_path / "camera_enabled.yaml"
    enabled_physics_file.write_text(
        yaml.safe_dump(simulation, sort_keys=False), encoding="utf-8"
    )

    urdf_root = _expand(
        URDF_FILE,
        geometry_file=GEOMETRY_FILE,
        simulation_physics_file=str(enabled_physics_file),
        gazebo_controllers_file=GAZEBO_CONTROLLERS_FILE,
        use_gazebo="true",
    )
    urdf_camera = urdf_root.find(
        "./gazebo[@reference='camera_link']/sensor[@name='camera']"
    )
    assert urdf_camera is not None
    assert math.isclose(
        float(urdf_camera.find("update_rate").text),
        simulation["sensors"]["camera"]["update_rate_hz"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        float(urdf_camera.find("./camera/horizontal_fov").text),
        simulation["sensors"]["camera"]["horizontal_fov_rad"],
        abs_tol=1e-9,
    )
    assert int(urdf_camera.find("./camera/image/width").text) == simulation["sensors"][
        "camera"
    ]["image_width_px"]
    assert int(urdf_camera.find("./camera/image/height").text) == simulation["sensors"][
        "camera"
    ]["image_height_px"]

    sdf_root = _expand(
        SDF_FILE,
        geometry_file=GEOMETRY_FILE,
        simulation_physics_file=str(enabled_physics_file),
        simulation_media_directory=MEDIA_DIRECTORY,
    )
    expected_topics = {
        "mobile_robot": "/sim/camera/image",
        "mobile_robot_2": "/robot_2/sim/camera/image",
    }
    for model_name, topic in expected_topics.items():
        camera = _sdf_model(sdf_root, model_name).find(
            "./link[@name='base_link']/sensor[@name='camera']"
        )
        assert camera is not None
        assert camera.find("topic").text == topic
        _assert_vector(
            _numbers(camera.find("pose").text),
            geometry["sensors"]["camera"]["xyz_m"]
            + geometry["sensors"]["camera"]["rpy_rad"],
        )
        assert math.isclose(
            float(camera.find("update_rate").text),
            simulation["sensors"]["camera"]["update_rate_hz"],
            abs_tol=1e-9,
        )
        assert math.isclose(
            float(camera.find("./camera/horizontal_fov").text),
            simulation["sensors"]["camera"]["horizontal_fov_rad"],
            abs_tol=1e-9,
        )
        assert int(camera.find("./camera/image/width").text) == simulation["sensors"][
            "camera"
        ]["image_width_px"]
        assert int(camera.find("./camera/image/height").text) == simulation["sensors"][
            "camera"
        ]["image_height_px"]


def test_nav2_footprint_contains_all_reference_collision_extents():
    geometry = _load_yaml(GEOMETRY_FILE)
    footprint_config = _load_yaml(FOOTPRINT_FILE)
    local_text = footprint_config["local_costmap"]["local_costmap"]["ros__parameters"][
        "footprint"
    ]
    global_text = footprint_config["global_costmap"]["global_costmap"]["ros__parameters"][
        "footprint"
    ]
    local_polygon = yaml.safe_load(local_text)
    assert local_polygon == yaml.safe_load(global_text)
    min_x = min(point[0] for point in local_polygon)
    max_x = max(point[0] for point in local_polygon)
    min_y = min(point[1] for point in local_polygon)
    max_y = max(point[1] for point in local_polygon)

    base_half_length = geometry["base"]["length_m"] / 2.0
    base_half_width = geometry["base"]["width_m"] / 2.0
    wheel_outer_y = (
        geometry["drive_wheels"]["center_separation_m"] / 2.0
        + geometry["drive_wheels"]["width_m"] / 2.0
    )
    camera = geometry["sensors"]["camera"]
    camera_half_length = camera["size_m"][0] / 2.0
    lidar = geometry["sensors"]["lidar"]
    caster = geometry["caster"]
    required_extents = {
        "min_x": min(
            -base_half_length,
            caster["xyz_m"][0] - caster["radius_m"],
            camera["xyz_m"][0] - camera_half_length,
            lidar["xyz_m"][0] - lidar["radius_m"],
        ),
        "max_x": max(
            base_half_length,
            caster["xyz_m"][0] + caster["radius_m"],
            camera["xyz_m"][0] + camera_half_length,
            lidar["xyz_m"][0] + lidar["radius_m"],
        ),
        "min_y": min(-base_half_width, -wheel_outer_y),
        "max_y": max(base_half_width, wheel_outer_y),
    }
    safety_margin_m = 0.02
    assert min_x <= required_extents["min_x"] - safety_margin_m
    assert max_x >= required_extents["max_x"] + safety_margin_m
    assert min_y <= required_extents["min_y"] - safety_margin_m
    assert max_y >= required_extents["max_y"] + safety_margin_m


def test_nominal_calibration_matches_reference_wheel_geometry():
    geometry = _load_yaml(GEOMETRY_FILE)
    calibration = _load_yaml(CALIBRATION_FILE)["chassis_driver_node"]["ros__parameters"]
    assert calibration["calibration_profile"] == "nominal_reference"
    assert math.isclose(
        calibration["wheel_diameter_m"],
        2.0 * geometry["drive_wheels"]["radius_m"],
        abs_tol=1e-9,
    )
    assert math.isclose(
        calibration["track_width_m"],
        geometry["drive_wheels"]["center_separation_m"],
        abs_tol=1e-9,
    )


@pytest.mark.parametrize(
    "field_path,bad_value",
    [
        (("base", "length_m"), -0.55),
        (("drive_wheels", "radius_m"), 0.0),
        (("frames", "base_link_height_m"), float("nan")),
        (("frames", "base_link_height_m"), 0.08),
        (("drive_wheels", "joint_z_m"), 0.005),
    ],
)
def test_invalid_geometry_is_rejected_during_xacro_expansion(field_path, bad_value):
    geometry = copy.deepcopy(_load_yaml(GEOMETRY_FILE))
    geometry[field_path[0]][field_path[1]] = bad_value
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", encoding="utf-8") as stream:
        yaml.safe_dump(geometry, stream)
        stream.flush()
        with pytest.raises(xacro.XacroException):
            _expand(URDF_FILE, geometry_file=stream.name)


def test_ground_contact_mismatch_is_rejected_by_sdf(tmp_path):
    geometry = copy.deepcopy(_load_yaml(GEOMETRY_FILE))
    geometry["drive_wheels"]["radius_m"] = 0.08
    geometry_file = tmp_path / "invalid_ground_contact.yaml"
    geometry_file.write_text(yaml.safe_dump(geometry), encoding="utf-8")

    with pytest.raises(xacro.XacroException):
        _expand(
            SDF_FILE,
            geometry_file=str(geometry_file),
            simulation_physics_file=PHYSICS_FILE,
            simulation_media_directory=MEDIA_DIRECTORY,
        )
