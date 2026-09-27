### Fixed: a rollout on a backend that cannot name its actions is refused, not run blind

Rollout entry points return `unsupported_by_backend` before building a policy or
stepping when a needed capability is absent (`joints` and `policy_rollout`;
`replay_episode` needs only `joints`, `start_policy` only `policy_rollout`). A
`NotImplementedError` from `robot_action_keys` or `set_robot_state_keys`, once
swallowed or raised, now returns that result before the first step, or
`unsupported_by_policy` naming the policy; other exceptions are unchanged. The
ROS 2 bridge skips `joint_states` without `joints`; teardown is never refused.
