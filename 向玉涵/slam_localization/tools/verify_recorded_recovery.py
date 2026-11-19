"""Use stored scans and real ROS nodes to exercise recovery at three map positions."""

from dataclasses import replace
import os
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools/map_review"))
from map_data import load_map, load_xyz
from measure_loop import rotate
import smoke_auto_relocalize as smoke


def main():
    saved = load_map(Path(sys.argv[1]))
    views = []
    for key in (int(arg) for arg in sys.argv[2:]):
        scan = load_xyz(replace(saved, pcd=saved.directory / f"keyframes/{key:06d}.pcd", point_count=None))
        rotation = rotate(saved.rotations[key], np.eye(3)).T
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        c, s = np.cos(yaw), np.sin(yaw)
        yaw_inverse = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
        leveled = scan @ (yaw_inverse @ rotation).T
        views.append((saved.positions[key].tolist(), float(yaw), leveled.astype(np.float32)))
        print(f"frame={key} map_xyz={saved.positions[key].tolist()} yaw_deg={np.degrees(yaw):.2f}",
              flush=True)
    smoke.scene = lambda: (None, views)
    smoke.write_pcd = lambda path, _: path.symlink_to(saved.pcd)
    os.environ["ROS_DOMAIN_ID"] = "178"
    smoke.main(force_loss_between_anchors=True)


if __name__ == "__main__":
    main()
