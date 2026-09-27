### Added: `ManipulationOptional` - a backend without manipulation refuses by contract

`strands_robots.simulation.capabilities` gains `ManipulationOptional`, a mixin
placed before `SimEngine` that declares only the core capabilities, answers
`add_object`, `remove_object` and `render` with an `unsupported_by_backend`
result, and makes `robot_joint_names` raise `CapabilityNotSupported`; the
exception and the `unsupported_result` builder are exported too. A class that
declares `objects`, `render` or `joints` while the refusal is still inherited
raises `TypeError` at class creation.
