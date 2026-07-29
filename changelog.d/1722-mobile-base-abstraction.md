### Changed: one drive contract for every mobile base - `MobileBaseRobot` + `Transport`

`RosBridgedRobot` and `RtpsRobot` were written by mirroring each other, so most
of each class was the same code: name validation, the `drive`/`stop` contract,
the `tools` property with suffix mangling, the `from_<preset>()` idiom. The
duplication was not free. It is why `RosBridgedRobot` - the class most likely to
be driving real ROS 2 hardware - shipped without the guards its newer siblings
had: `drive(linear=float("nan"))` published NaN straight onto `cmd_vel` (a
`min`/`max` clamp passes `nan` through silently), `drive(duration=float("inf"))`
became an unbounded publish loop, and a timed command left the robot moving
because nothing sent the trailing zero.

`MobileBaseRobot` now owns the invariant half - validation, the drive contract
and its safety semantics, the `init_services` enable handshake, `get_pose` /
`get_scan`, and the `tools` property. A robot class supplies only what varies:
a `Transport` (how bytes move) and, when the platform is not differential-drive,
a `_cmd_fields` override (what the command message looks like - the kinematics
seam). `Transport` requires `publish` and `echo` only; `service_call` and
`action_send_goal` are separate optional-capability protocols, because
`use_rtps` has neither and a protocol forcing it to declare them would make it
lie. The base asks (`robot.supports(...)`) and reflects the answer: an
`init_services` handshake wired onto a transport that cannot call services is
refused at construction rather than on the track, and tools are built from what
is actually wired, so an agent is never handed one that can only answer "not
configured".

Two behavioural changes, both closing gaps. `RosBridgedRobot` inherits the
hardened contract above. It also gains a `stop_<node>` agent tool: it previously
exposed `drive` with no `stop`, so an agent's only way to halt was to infer
`drive(0, 0)`, which is not discoverable from the tool list it is given.

Speed and duration limits stay unset by default on both ported classes - neither
knows the limits of the third-party robot it drives, and inventing one would
silently cap an existing caller. A limit left `None` means "this platform
declares no limit", never zero.
