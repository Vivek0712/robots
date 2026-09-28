### Fixed: a hand-written user overlay key that no lookup can reach is refused

`loader._merge_user_robots` merged `user_robots.json` keys verbatim, but every
reader looks a robot up by its `normalize_robot_name` fold (lowercase, trimmed,
dashes as underscores). `register_robot` folds before it writes, so only an
overlay written by hand or by another tool escaped: a key like `rover-001` or
`My_Arm` loaded without complaint and then answered no query, not even its own
spelling. The merge now refuses an overlay key that is not its own fold with a
`ValueError` that names the overlay file and the spelling to rename it to, and
warns when that spelling already belongs to another robot that renaming would
replace. The key is refused rather than folded, because folding could collapse
two keys onto one entry and keep whichever merged last. The refusal fails the
registry load, as an alias collision does, so every lookup raises until the key
is renamed (or removed with `unregister_robot`). `unregister_robot` now removes
a key spelled exactly as its argument before it falls back to the folded
spelling, so it can remove such a key.
