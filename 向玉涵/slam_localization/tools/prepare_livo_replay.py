"""Generate an isolated replay config; never activate production TF/calibration."""
import argparse
import hashlib
import json
from pathlib import Path

import yaml
import numpy as np

from robocon_core.livo_config import parameters
from derive_rig_extrinsics import derive

ROOT = Path(__file__).resolve().parents[1]


def mechanical_seed():
    """Recompute the already documented nominal; never call this a measured calibration."""
    draft_path = ROOT / "config/design/mid360_d435i_rig_nominal.yaml"
    draft = yaml.safe_load(draft_path.read_text(encoding="utf-8"))
    factory_path = ROOT / draft["factory_evidence"]
    factory = json.loads(factory_path.read_text(encoding="utf-8"))
    dimensions = draft["required_measurements_mm"]
    result = derive(draft, factory, dimensions["front_glass_x_mm"],
                    dimensions["ir_center_height_mm"], draft["nominal_camera_mount_y_mm"])
    lio_path = ROOT / "src/robocon_bringup/config/fast_lio_mid360.yaml"
    mapping = yaml.safe_load(lio_path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]["mapping"]
    T_I_L = np.eye(4)
    T_I_L[:3, :3] = np.asarray(mapping["extrinsic_R"]).reshape(3, 3)
    T_I_L[:3, 3] = mapping["extrinsic_T"]
    return {"camera_serial": result["camera_serial"], "T_C_L": result["T_C_L"],
            "T_I_L": T_I_L.tolist(), "calibration_id": "mechanical-seed-20261005-unverified",
            "status": "mechanical_initialization_only", "residual_time_verified": False,
            "camera_to_common_time_s": None}, [draft_path, factory_path, lio_path]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-info", required=True, type=Path)
    calibration_source = parser.add_mutually_exclusive_group(required=True)
    calibration_source.add_argument("--calibration", type=Path)
    calibration_source.add_argument("--mechanical-seed", action="store_true",
                                    help="Recompute the documented unmeasured rig initial guess")
    parser.add_argument("--output", required=True, type=Path, help="New directory")
    parser.add_argument("--allow-unverified", action="store_true",
                        help="Mechanical seed/unknown residual time, isolated replay ONLY")
    args = parser.parse_args()
    camera = json.loads(args.camera_info.read_text(encoding="utf-8"))
    if args.mechanical_seed:
        calibration, sources = mechanical_seed()
    else:
        calibration = yaml.safe_load(args.calibration.read_text(encoding="utf-8"))
        sources = [args.calibration]
    config, evidence = parameters(camera, calibration, allow_unverified=args.allow_unverified)
    evidence["source_sha256"] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                                 for p in [args.camera_info, *sources]}
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "livo.yaml").write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    (args.output / "camera.json").write_text(json.dumps(camera, indent=2) + "\n", encoding="utf-8")
    evidence["generated_sha256"] = {
        name: hashlib.sha256((args.output / name).read_bytes()).hexdigest()
        for name in ("livo.yaml", "camera.json")}
    (args.output / "evidence.json").write_text(json.dumps(evidence, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(evidence, indent=2))


if __name__ == "__main__":
    main()
