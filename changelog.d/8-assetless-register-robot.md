### Added: `register_robot` registers a robot with no simulation asset

`model_xml` is now optional. Without it the entry is stored with no `asset`
block, and it must declare `hardware` with a non-empty `lerobot_type` or
`driver` `"strands"`, so the entry declares how it is driven for real (a
`strands` driver still has to be registered with `register_native_driver`):

```python
register_robot("drone", category="aerial", hardware={"driver": "strands"})
```

`scene_xml`, `asset_dir` and `robot_descriptions_module` only describe an asset
and are refused without `model_xml`. `Robot("drone", mode="sim")` reports the
robot as real-hardware only. Registration with `model_xml` is unchanged.
