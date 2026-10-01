# Deployed aircraft profiles

This directory records source and configuration captured from the two deployed
aircraft. Files that are byte-identical on both aircraft are synchronized to the
normal repository paths. Files that differ, or exist on only one aircraft, are
stored below `nx163/overrides` and `nx164/overrides` with their original paths.

The canonical `uav_ros2_project/src/uav_recon/config/recon.yaml` remains the
NX163 profile for backward compatibility. Always apply the matching host
override when deploying to NX164. The systemd service files are snapshots of
the active host-specific user, home and startup command; they are not portable
between aircraft without substituting those paths.

Runtime logs, recordings, model binaries, build/install trees, caches and
temporary state are intentionally excluded.
