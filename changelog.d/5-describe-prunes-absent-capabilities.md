### Added: `describe()` omits the methods of an absent capability

`describe()` on the base `SimEngine` and the MuJoCo, Newton and Isaac backends
no longer advertises a method whose capability the backend lacks, unless the
backend implements that method itself (a joint-less backend's own
`robot_action_keys` stays listed); what is dropped is the base implementation or
the `ManipulationOptional` refusal of: `joints`
(`robot_joint_names`, `robot_action_keys`, `set_joint_positions`,
`set_joint_velocities`), `objects` (`add_object`, `remove_object`,
`list_objects`, `move_object`), `render` (`render`, `render_depth`,
`render_all`), `policy_rollout` (`run_policy`, `start_policy`, `eval_policy`,
`evaluate_benchmark`, `replay_episode`, `run_multi_policy`) and each optional
capability's own method. Backends that declare nothing are unaffected.

The check that a declared capability is not backed by a `ManipulationOptional`
refusal now also sees through `functools.partial`/`partialmethod` and applies to
a subclass that inherits the declaration and reverts the member.
