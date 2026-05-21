# Mapping, Localization, Nav2

Use existing Waveshare wrappers first: `ugv_slam` for Cartographer/GMapping/RTAB-Map and `ugv_nav`
for Nav2. Waver launch files include these packages instead of modifying them.

Do not patrol from an unvalidated map. Outdoor maps must be checked after rain, moved objects,
vegetation changes, and lighting/sensor changes. GPS/RTK is required for geographic outdoor patrol;
without it, Waver supports local map-frame waypoints only.
