### Added: register_robot registers a robot with no simulation asset

`register_robot` required `model_xml` and refused a robot whose asset directory
or model file was missing, so a robot with no mesh - a spacecraft, a remote
service robot - could not be added through the public writer even though the
package registry already ships hardware-only entries.

`model_xml` is now optional. Without it the entry is stored with no `asset`
block, and it must declare `hardware` with a non-empty `lerobot_type` or a
`driver` other than `"auto"`, so the robot is reachable in real mode:

```python
register_robot("orbiter", category="aerial", hardware={"driver": "strands"})
```

`scene_xml`, `asset_dir` and `robot_descriptions_module` only describe an asset
and are refused without `model_xml`. `Robot("orbiter", mode="sim")` reports the
robot as real-hardware only. Registration with `model_xml` is unchanged.
