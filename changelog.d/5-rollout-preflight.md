### Fixed: a rollout on a backend that cannot name its actions is refused, not run blind

`run_policy`, `eval_policy`, `run_multi_policy` and `evaluate_benchmark` now
return the `unsupported_by_backend` result before a policy is built or the world
steps when the backend lacks the `joints` or `policy_rollout` capability;
`replay_episode` does the same without `joints`, and `start_policy` (which
delegates to `run_policy`) without `policy_rollout`.
A `NotImplementedError` (including `CapabilityNotSupported`) from
`robot_action_keys` or a policy's `set_robot_state_keys` was swallowed by
`run_policy` and raised from `eval_policy` and `evaluate_benchmark`; all three
now return that result before the first step, naming the member in
`json.member`; when the policy raised, the code is `unsupported_by_policy` and
the result names the policy. Any other exception there keeps
its previous handling. A backend whose own `capabilities` member has another
shape is checked against the derived set. The ROS 2 bridge skips `joint_states`
for a backend without `joints` instead of logging a failure every step.
`stop_policy`, `destroy` and `reset` are never refused.
