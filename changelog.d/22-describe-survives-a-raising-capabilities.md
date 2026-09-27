### Fixed: `describe()` and the rollout preflight survive a backend's own `capabilities` that raises

A third-party backend that already defines a `capabilities` member which raises
(not only one of another shape) made `describe()` - the entry point an agent
calls first - raise too. `describe()` now reports `"capabilities": None` for
it, and the rollout preflight falls back to the derived capability set, as they
already did for a member that returns something other than a set of names.
