### Fixed: a `str` subclass cannot pass a blank `model_xml` to `register_robot`

The blank `model_xml` check calls `str.strip` rather than the value's own
`strip`, as the `lerobot_type` check already does, so a `str` subclass that
overrides `strip` is refused the same as a plain blank string.
