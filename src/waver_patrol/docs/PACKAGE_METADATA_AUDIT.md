# Package Metadata Audit

This audit records package metadata state without inventing license claims.

| package | current description/license status | action taken | unresolved items |
| --- | --- | --- | --- |
| `waver_patrol` | Waver-owned patrol/autonomy package, Apache-2.0 declared. | No metadata change needed. | Keep maintainer current before release. |
| `waver_experiment_logger` | Waver-owned Gazebo bird patrol logger, Apache-2.0 declared, placeholder maintainer email. | Documented placeholder maintainer. | Replace `you@example.com` with project maintainer email before public release. |
| `waver_seo_tracking` | Waver-owned Gazebo-only SEO tracking adapters, Apache-2.0 declared, placeholder maintainer email. | Documented Gazebo-only scope. | Replace `you@example.com` before public release. |
| `livox_laser_simulation_RO2` | Upstream/vendor-like package with TODO description and TODO license. | No license changed. | Resolve upstream license before redistribution. |
| `livox_ros_driver2` | Upstream Livox driver, MIT declared. | Preserved upstream metadata. | Check submodule/vendor release before packaging. |
| `ugv_main/*` | Waveshare/UGV platform packages. | No license overwritten. | Audit vendor provenance before public source archive. |
| `ugv_else/*` | Third-party ROS packages. | No license overwritten. | Keep upstream license files with any redistributed archive. |

## Policy

- Do not overwrite upstream licenses casually.
- Do not guess legal license terms.
- For Waver-owned packages, descriptions should say whether a package is real,
  Gazebo-only, evaluation-only, or operator UI.
