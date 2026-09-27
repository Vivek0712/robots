### Added: `SimEngine.capabilities()` - a backend states what it supports

The new `strands_robots.simulation.capabilities` module names a closed
capability vocabulary (plus `vendor:name` extras), the `unsupported_by_backend`
code and `check_capabilities`, which accepts any object with a `capabilities()`
method.
`SimEngine` gains a `CAPABILITIES` declaration, validated and frozen at class
creation, and a `capabilities()` that existing backends derive unchanged;
`describe()` gains a `capabilities` key (`None` when a backend's own
`capabilities` cannot answer). Both new members can clash with an
out-of-tree backend that already defines either name.
