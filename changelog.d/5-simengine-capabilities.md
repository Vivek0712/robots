### Added: `SimEngine.capabilities()` - a backend states what it supports

The new `strands_robots.simulation.capabilities` module names a closed
capability vocabulary (plus `vendor:name` extras), the `unsupported_by_backend`
code, `CapabilityNotSupported`, `unsupported_result` and `check_capabilities`.
`SimEngine` gains a `CAPABILITIES` declaration, validated and frozen at class
creation, and a `capabilities()` that existing backends derive unchanged;
`describe()` gains a `capabilities` key. Both new members can clash with an
out-of-tree backend that already defines either name.
