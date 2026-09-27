### Added: `describe()` omits the methods of an absent capability

`describe()` on the base `SimEngine` and the MuJoCo, Newton and Isaac backends
no longer advertises a method whose capability the backend lacks: `joints`
(`robot_joint_names`, `robot_action_keys`, `set_joint_positions`,
`set_joint_velocities`), `objects` (`add_object`, `remove_object`,
`list_objects`, `move_object`), `render` (`render`, `render_depth`,
`render_all`), `policy_rollout` (`run_policy`, `start_policy`, `eval_policy`,
`evaluate_benchmark`, `replay_episode`, `run_multi_policy`) and each optional
capability's own method. Backends that declare nothing are unaffected.
