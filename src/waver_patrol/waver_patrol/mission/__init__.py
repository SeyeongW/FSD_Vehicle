"""Mission-level Waver nodes.

This package keeps radar/object mission orchestration inside `waver_patrol`
instead of introducing another ROS package. None of these nodes publish
`/cmd_vel` directly; movement is requested through Nav2 actions or candidate
topics that are filtered by `safety_cmd_mux_node`.
"""
