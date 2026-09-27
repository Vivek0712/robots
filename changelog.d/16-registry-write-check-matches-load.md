### Fixed: `register_robot` refuses what the next load would refuse, and a non-`str` `model_xml`

The write-time check `register_robot` runs before it persists now applies the
loader's unfolded-key refusal too, so `register_robot(..., overwrite=True)` can
no longer write into a `user_robots.json` whose hand-written unfolded key makes
every read fail. A `model_xml` that is not a `str` (a `Path`, an `int`) is a
`TypeError` naming the type rather than an `AttributeError`, and a blank
`lerobot_type` is detected with `str.strip` so a `str` subclass cannot hide it.
