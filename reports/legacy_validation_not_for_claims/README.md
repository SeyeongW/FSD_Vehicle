# Legacy Validation Logs

Historical development logs only. Do not cite as paper result unless revalidated
with the current schema.

Current-schema validation summaries must include:

- `raw_status`
- `final_status`
- `limitation`
- `command_rc`
- `evidence_path`
- `key_output`

Older `summary.csv` files under local report folders may contain skips,
timeouts, missing Gazebo/Livox plugins, or stale command outputs. They are
excluded from `source_release` archives by default and must not be interpreted
as PASS evidence.
