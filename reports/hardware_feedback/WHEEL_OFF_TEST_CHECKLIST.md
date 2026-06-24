# Wheel-Off Test Checklist

This checklist is for a human operator. Do not automate it.

1. Lift the Waver or disconnect drive wheels from the ground.
2. Keep physical E-stop within reach.
3. Power the Jetson and Waver using the validated field wiring.
4. Use stable serial path:
   `/dev/serial/by-id/<CP2102N_WAVER_SERIAL_ID>`.
5. Verify ROS domain and workspace on Jetson.
6. Confirm final `/cmd_vel` publisher is `safety_cmd_mux_node`.
7. Confirm serial owner is only `waver_base_driver_node`.
8. Confirm voltage matches OLED/multimeter after applying `voltage_scale`.
9. Start in STANDBY and check `/cmd_vel` is zero.
10. Send tiny manual W/A/S/D commands from the UI.
11. Confirm wheel direction:
    - forward/backward
    - pivot left/right
    - release-to-stop timeout
12. Press E-stop and confirm immediate zero command.
13. Save logs and notes under `reports/hardware_feedback/`.

