### Added: `SimEngine.capabilities()` - a backend states what it supports

The new `strands_robots.simulation.capabilities` module names a closed
capability vocabulary (plus `vendor:name` extras), the `unsupported_by_backend`
code, `CapabilityNotSupported`, `unsupported_result` and `check_capabilities`.
`SimEngine` gains an optional `CAPABILITIES` declaration, checked at class
creation, and a concrete `capabilities()` that existing backends derive
unchanged; `describe()` reports it and omits the methods of an absent one.
