### Fixed: `AckermannRosRobot.from_deepracer()` arms the car in manual mode

The preset's `vehicle_state` handshake sent `state=1`, which on the stock
DeepRacer `ctrl_pkg` is autonomous mode: the car accepted the call, reported
success, and then dropped every manual-drive servo message because that mode
runs the on-car inference loop instead. The preset now sends `state=0` (manual;
1 is autonomous, 2 is calibration), the only mode in which `ctrl_pkg` relays
`/webserver_pkg/manual_drive` to the servo node. Verified against a stock
`aws-deepracer-core 2.0.113.0` (Foxy) car: with state 0 the relayed throttle
appears on `/ctrl_pkg/servo_msg` and the PWM duty cycle follows it. The
docstring and the pinned-wiring tests say which mode each number is, and a new
test fails if the handshake ever selects a non-manual mode again.
