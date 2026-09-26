### Fixed: a hand-written user overlay key that no lookup can reach is refused

`loader._merge_user_robots` merged `user_robots.json` keys verbatim, but every
reader looks a robot up by its `normalize_robot_name` fold (lowercase, trimmed,
dashes as underscores). `register_robot` folds before it writes, so only an
overlay written by hand or by another tool escaped: a key like `sat-001` or
`My_Arm` loaded without complaint and then answered no query, not even its own
spelling. `_validate_robots` now refuses a robot key that is not its own fold
with a `ValueError` that names the spelling to rename it to, and says when that
spelling already belongs to another robot. The key is refused rather than
folded, because folding could collapse two keys onto one entry and keep
whichever merged last.
