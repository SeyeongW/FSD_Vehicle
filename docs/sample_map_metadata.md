# Sample Map Metadata

The repository contains small map fixtures under `maps/` so Gazebo/UI smoke
commands can start with a known map path. These files are sample fixtures, not
field-validated real-site maps.

| File | Type | Source | Resolution | Intended use |
|---|---|---|---|---|
| `maps/waver_latest_map.yaml` | sample map YAML | local Gazebo/SLAM smoke output | see YAML | UI/map-load smoke tests |
| `maps/waver_latest_map.pgm` | sample occupancy image | local Gazebo/SLAM smoke output | see YAML | UI/map-load smoke tests |
| `maps/archive/*` | archived sample map copies | local Gazebo/SLAM smoke output | see YAML | local historical comparison only; excluded from `source_release` |

Paper or field reports must not describe these maps as real-world survey maps.
Regenerate and document a new map for each real deployment site.
