# Upstream sources

`3d_bbs`: [KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs), revision
`41529a34a2fb9618b5ff560fb3c2363f1615666d`, MIT license retained.
Only the CPU branch and its required headers are included. Changes:

- Use unsigned voxel hash arithmetic to avoid signed integer overflow.
- Use the standard library hash instead of adding Boost for integer vectors.
- Use `steady_clock` for time limits and a portable pi expression.
- Retain distinct high-score pose regions for GICP ambiguity checking. Regions
  within five percentage points of the best voxel score are considered. Nearby
  poses are grouped using the existing fine-registration basin. More than 16
  candidate regions gives an ambiguous result rather than an arbitrary pose.

`fastlio2_ros2_pgo`: [liangheming/FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2),
revision `7baa59f7339ba88ce3b646fe35b23f61031c5d3f`, MIT license retained.
Only the pose-graph core is included. The ROS node is this project's adapter.

- Keep the keyframe and odometry-factor construction and the iSAM2 update.
- Search older keyframe clouds with the vendored CPU 3D-BBS. This removes the
  distance gate on the current odometry estimate. One background worker
  returns a loop relative to its historical anchor.
- Verify candidates with whole-scan GICP, reject competing fits, and check
  point-to-plane observability before adding the loop factor.
- Use Huber loss for loop factors and a positive numerical variance floor.
- The indoor adapter uses a 10 s loop separation; upstream's 60 s default is
  retained in the library itself. Mapping and fixed-map localization are separate
  `map -> odom` owners.

Pinned downloads, original files and hashes are stored in the dated DICodex
algorithm-review directory. The originals there are kept for source comparison.
