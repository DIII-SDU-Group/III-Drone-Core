import importlib.util
from pathlib import Path
import sys
import yaml


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "tf.launch.py"
CONFIGURATION_SOURCE = LAUNCH_FILE.parents[2] / "III-Drone-Configuration"
sys.path.insert(0, str(CONFIGURATION_SOURCE))


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("iii_drone_core_tf_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_hil_keeps_px4_pose_but_uses_simulated_payload_extrinsics(monkeypatch):
    module = _load_launch_module()
    monkeypatch.setenv("III_SYSTEM_PROFILE", "hil")

    profile = module._runtime_profile()

    assert profile == "hil"
    assert module._static_transform_prefix(profile) == "/tf/sim"


def test_hil_launch_contains_only_the_dynamic_px4_tf_publisher(monkeypatch, tmp_path):
    module = _load_launch_module()
    monkeypatch.setenv("III_SYSTEM_PROFILE", "hil")
    params_file = tmp_path / "params.yaml"
    params_file.write_text(
        yaml.safe_dump({
            "/**": {"ros__parameters": {
                "/tf/drone_frame_id": "drone",
                "/tf/cable_gripper_frame_id": "cable_gripper",
                "/tf/mmwave_frame_id": "mmwave",
                "/tf/sim/drone_to_cable_gripper": [0, 0, 0, 0, 0, 0],
                "/tf/sim/drone_to_mmwave": [0, 0, 0, 0, 0, 0],
            }}
        }),
        encoding="utf-8",
    )
    monkeypatch.setattr(module, "_resolve_ros_params_file", lambda: str(params_file))
    monkeypatch.setattr(module, "_parameter_sources", lambda: [])

    entities = module.generate_launch_description().entities
    nodes = [entity for entity in entities if hasattr(entity, "node_package")]

    assert [(node.node_package, node.node_executable) for node in nodes] == [
        ("iii_drone_core", "drone_frame_broadcaster"),
    ]


def test_aircraft_profiles_use_physical_payload_extrinsics(monkeypatch):
    module = _load_launch_module()

    for profile in ("real", "opti_track"):
        monkeypatch.setenv("III_SYSTEM_PROFILE", profile)
        selected = module._runtime_profile()
        assert selected == profile
        assert module._static_transform_prefix(selected) == "/tf"


def test_unknown_profile_fails_closed_to_real_extrinsics(monkeypatch):
    module = _load_launch_module()
    monkeypatch.setenv("III_SYSTEM_PROFILE", "unexpected")

    assert module._runtime_profile() == "real"
