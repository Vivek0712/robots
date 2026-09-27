### Fixed: a rollout on a backend that cannot name its actions is refused, not run blind

`run_policy`, `start_policy`, `eval_policy`, `run_multi_policy` and
`evaluate_benchmark` now return the `unsupported_by_backend` result before a
policy is built or the world steps when the backend lacks the `joints` or
`policy_rollout` capability; `replay_episode` does the same without `joints`.
A `NotImplementedError` (including `CapabilityNotSupported`) from
`robot_action_keys` or a policy's `set_robot_state_keys` is no longer swallowed:
the rollout returns that result, naming the member in `json.member`, before its
first step. Any other exception there keeps its previous handling. The ROS 2
bridge skips `joint_states` for a backend without `joints` instead of logging a
failure every step. `stop_policy`, `destroy` and `reset` are never refused.
