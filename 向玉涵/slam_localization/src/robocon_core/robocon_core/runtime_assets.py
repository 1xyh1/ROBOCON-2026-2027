"""Versioned runtime identity and map/calibration bundle validation.

Rulebook dimensions and SDK defaults are not calibration or surveyed maps.
Hashes and review records catch accidental asset mix-ups; they are not a
cryptographic proof of independent approval or of localization accuracy.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Any

import yaml


LOCALIZATION_LIMITS = {
    "maximum_rmse_m": 0.08,
    "minimum_overlap": 0.55,
    "minimum_3d_geometry_ratio": 0.003,
}


@dataclass(frozen=True)
class Bundle:
    identity: str
    manifest: Path
    digest: str
    data_files: tuple[Path, ...]


@dataclass(frozen=True)
class RuntimeProfile:
    path: Path
    mode: str
    robot_id: str
    camera_serial: str
    mid360_ip: str
    mid360_host_ip: str
    mid360_interface: str
    ptp_utc_offset_s: int
    global_localization_enabled: bool
    map_bundle: Bundle | None
    calibration_bundle: Bundle | None
    camera_enabled: bool = True


def sha256(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            result.update(chunk)
    return result.hexdigest()


def _mapping(value: Any, label: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _nonempty(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be nonempty")
    return value.strip()


def _digest(value: Any, label: str) -> str:
    digest = _nonempty(value, label).lower()
    if len(digest) != 64 or any(letter not in "0123456789abcdef" for letter in digest):
        raise ValueError(f"{label} is not a SHA256 digest")
    return digest


def _bundle_file(manifest: Path, name: Any, label: str) -> Path:
    relative = Path(_nonempty(name, label))
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"{label} path must stay inside its bundle")
    target = (manifest.parent / relative).resolve()
    if not target.is_relative_to(manifest.parent):
        raise ValueError(f"{label} path must stay inside its bundle")
    return target


def _bundle(value: Any, base: Path, label: str) -> Bundle | None:
    if value is None:
        return None
    reference = _mapping(value, label)
    identity = _nonempty(reference.get("id"), label + ".id")
    manifest = (base / _nonempty(reference.get("manifest"), label + ".manifest")).resolve()
    expected = _digest(reference.get("sha256"), label + ".sha256")
    if not manifest.is_file() or sha256(manifest) != expected:
        raise ValueError(f"{label} manifest is missing or its SHA256 differs")
    contents = _mapping(yaml.safe_load(manifest.read_text(encoding="utf-8")), label + " manifest")
    if (contents.get("schema_version") != 1 or contents.get("bundle_type") != label
            or contents.get("id") != identity or contents.get("approved") is not True):
        raise ValueError(f"{label} schema, type, identity or approval flag is invalid")
    approval = _mapping(contents.get("approval"), label + " approval")
    _nonempty(approval.get("reviewer"), label + " approval.reviewer")
    evidence = _bundle_file(manifest, approval.get("evidence"), label + " approval.evidence")
    evidence_digest = _digest(approval.get("sha256"), label + " approval.sha256")
    if not evidence.is_file() or sha256(evidence) != evidence_digest:
        raise ValueError(f"{label} approval evidence is missing or changed")
    files = _mapping(contents.get("files"), label + " files")
    if not files:
        raise ValueError(f"{label} must list its measured data files")
    resolved_files = []
    for name, digest in files.items():
        target = _bundle_file(manifest, name, label + " file")
        if not target.is_file() or sha256(target) != _digest(digest, label + " file SHA256"):
            raise ValueError(f"{label} data file missing or changed: {name}")
        resolved_files.append(target)
    if evidence in resolved_files:
        raise ValueError(f"{label} approval evidence cannot be measured data")
    return Bundle(identity, manifest, expected, tuple(resolved_files))


def load_runtime_profile(path: Path) -> RuntimeProfile:
    path = path.resolve()
    data = _mapping(yaml.safe_load(path.read_text(encoding="utf-8")), "runtime profile")
    if data.get("schema_version") != 1:
        raise ValueError("unsupported runtime profile schema")
    mode = _nonempty(data.get("mode"), "mode")
    if mode not in {"practice", "replay", "competition"}:
        raise ValueError("invalid runtime mode")
    sensors = _mapping(data.get("sensors"), "sensors")
    camera = _mapping(sensors.get("d435i"), "sensors.d435i")
    camera_enabled = camera.get("enabled", True)
    if not isinstance(camera_enabled, bool):
        raise ValueError("d435i.enabled must be boolean")
    lidar = _mapping(sensors.get("mid360"), "sensors.mid360")
    localization = _mapping(data.get("localization"), "localization")
    enabled = localization.get("enabled")
    if not isinstance(enabled, bool):
        raise ValueError("localization.enabled must be boolean")
    map_bundle = _bundle(localization.get("map"), path.parent, "map")
    calibration_bundle = _bundle(localization.get("calibration"), path.parent, "calibration")
    if enabled and (map_bundle is None or calibration_bundle is None):
        raise ValueError("global localization requires approved map and calibration bundles")
    if map_bundle is not None and calibration_bundle is not None:
        if map_bundle.manifest == calibration_bundle.manifest or map_bundle.identity == calibration_bundle.identity:
            raise ValueError("map and calibration must have distinct manifests and identities")
        if set(map_bundle.data_files) & set(calibration_bundle.data_files):
            raise ValueError("map and calibration cannot share measured data files")
    if mode == "competition" and not enabled:
        raise ValueError("competition profile cannot disable verified global localization")
    offset = lidar.get("ptp_utc_offset_s")
    if not isinstance(offset, int) or isinstance(offset, bool) or not 0 < offset < 100:
        raise ValueError("invalid MID-360 PTP/UTC offset")
    return RuntimeProfile(
        path=path, mode=mode, robot_id=_nonempty(data.get("robot_id"), "robot_id"),
        camera_serial=_nonempty(camera.get("serial"), "d435i.serial") if camera_enabled else "",
        mid360_ip=_nonempty(lidar.get("ip"), "mid360.ip"),
        mid360_host_ip=_nonempty(lidar.get("host_ip"), "mid360.host_ip"),
        mid360_interface=_nonempty(lidar.get("interface"), "mid360.interface"),
        ptp_utc_offset_s=offset, global_localization_enabled=enabled,
        map_bundle=map_bundle, calibration_bundle=calibration_bundle,
        camera_enabled=camera_enabled,
    )


def _validate_lio_config(path: Path) -> None:
    parameters = _mapping(yaml.safe_load(path.read_text(encoding='utf-8')), 'FAST-LIO parameters')
    lio_parameters = _mapping(_mapping(parameters.get('/**'), 'FAST-LIO /**').get('ros__parameters'), 'FAST-LIO ros__parameters')
    contract = {
        'common': {'lid_topic': '/livox/lidar', 'imu_topic': '/livox/imu',
                   'odom_frame': 'odom', 'body_frame': 'mid360_imu', 'publish_tf': True},
        'publish': {'scan_publish_en': True, 'scan_bodyframe_pub_en': True},
    }
    for group, fields in contract.items():
        actual = _mapping(lio_parameters.get(group), 'FAST-LIO ' + group)
        for key, expected in fields.items():
            value = actual.get(key)
            if type(value) is not type(expected) or value != expected:
                raise ValueError(f'FAST-LIO contract {group}.{key} requires {expected!r}; got {value!r}')


def localization_launch_assets(profile: RuntimeProfile) -> dict:
    """Resolve the map and frontend files consumed by global localization."""
    if profile.map_bundle is None or profile.calibration_bundle is None:
        raise ValueError('Map localization requires map and calibration bundles')
    map_bundle, calibration = profile.map_bundle, profile.calibration_bundle
    map_data = yaml.safe_load(map_bundle.manifest.read_text(encoding='utf-8'))
    calibration_data = yaml.safe_load(calibration.manifest.read_text(encoding='utf-8'))
    pcd = _bundle_file(map_bundle.manifest, map_data.get('pcd_file'), 'map.pcd_file')
    lio = _bundle_file(calibration.manifest, calibration_data.get('fast_lio_config'), 'calibration.fast_lio_config')
    if pcd not in map_bundle.data_files or pcd.suffix != '.pcd':
        raise ValueError('map.pcd_file must select a hashed PCD in the map bundle')
    if lio not in calibration.data_files or lio.suffix not in {'.yaml', '.yml'}:
        raise ValueError('calibration.fast_lio_config must select a hashed ROS parameter file')
    _validate_lio_config(lio)
    floor = _nonempty(map_data.get('floor'), 'map.floor')
    if floor not in {'ground', 'l1', 'l2'}:
        raise ValueError('Invalid map floor')
    global_parameters = {'map_pcd': str(pcd), 'map_id': map_bundle.identity,
        'calibration_id': calibration.identity, 'floor': floor, **LOCALIZATION_LIMITS}
    executive_parameters = {'expected_map_id': map_bundle.identity,
        'expected_calibration_id': calibration.identity, 'initial_floor': floor,
        'maximum_rmse_m': LOCALIZATION_LIMITS['maximum_rmse_m'],
        'minimum_overlap_ratio': LOCALIZATION_LIMITS['minimum_overlap'],
        'minimum_geometry_ratio': LOCALIZATION_LIMITS['minimum_3d_geometry_ratio']}
    return {'lio_config_file': str(lio), 'parameters': {
        'global_localizer': {'ros__parameters': global_parameters},
        'mission_executive': {'ros__parameters': executive_parameters}}}


def standalone_localization_arguments(folder: Path, config_dir: Path, state: Path, *,
                                      lio_config: Path | None = None,
                                      livox_config: Path | None = None) -> dict[str, str]:
    """Prepare the fixed-map arguments shared by direct launch and bag replay."""
    info = yaml.safe_load((folder / 'map.yaml').read_text(encoding='utf-8'))
    if info['frame'] != 'map' or info['up_axis'] != 'z':
        raise ValueError(f'{folder} must use the Z-up map frame')
    pcd = folder / info['pcd_file']
    lio = lio_config or folder / 'fast_lio.yaml'
    _validate_lio_config(lio)
    if livox_config is None:
        livox_config = folder / 'livox.json'
        if not livox_config.is_file():
            livox_config = config_dir / 'livox_mid360_mini_host.json'
    network = json.loads(livox_config.read_bytes())
    parameters = {'global_localizer': {'ros__parameters': {
        'map_pcd': str(pcd), 'map_id': 'map-' + sha256(pcd)[:16],
        'calibration_id': 'lio-config-' + sha256(lio)[:16], 'floor': 'ground',
        **LOCALIZATION_LIMITS,
    }}}
    state.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile(mode='w', dir=state, prefix='fixed-map-', suffix='.yaml',
                            encoding='utf-8', delete=False) as runtime:
        yaml.safe_dump(parameters, runtime)
    return {
        'localization_config': runtime.name,
        'checkpoint_path': str(state / 'checkpoint.json'),
        'lio_config_file': str(lio),
        'livox_config': str(livox_config),
        'mid360_ip': network['lidar_configs'][0]['ip'],
        'mid360_host_ip': network['MID360']['host_net_info']['cmd_data_ip'],
        'standalone_localization': 'true',
    }
