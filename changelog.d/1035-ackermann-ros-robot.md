### Added: `AckermannRosRobot` - Ackermann ROS 2 cars (AWS DeepRacer) as strands robots

`RosBridgedRobot` covers differential-drive bases (Twist on `cmd_vel` +
odometry); Ackermann cars expose neither. The new bridge keeps the same
`drive(linear, angular, duration)` agent contract and converts through a
bicycle model to normalized servo commands (`ServoCtrlMsg` on the DeepRacer),
runs a declarative `init_services` handshake once before the first command
(the DeepRacer's manual-mode two-step, preconfigured in `from_deepracer()`),
clamps speed, rejects over-long holds loudly, and always trails timed
commands with a zero servo message so a timed drive cannot leave the car
driving (a bare single-shot command latches until stop, matching raw servo
semantics). Conditional `get_scan` tool; no `get_pose` (the stock platform has
no odometry).

`drive` and the constructor limits validate through the same
`finite_number_error`, `positive_finite_number_error` and
`positive_whole_number_error` domains the three differential-drive bridges
call, so the fourth transport refuses an unusable velocity, hold or count with
byte-identical text rather than a hand-rolled copy of the contract.
