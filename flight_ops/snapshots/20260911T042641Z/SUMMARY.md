# Snapshot 20260911T042641Z -- droneA-flown-config

## Sources

| source | status | artifact / error |
|---|---|---|
| `git` | ok | git.txt |
| `env` | ok | env.txt |
| `apt` | MISSING | dpkg unavailable or no ros-humble packages |
| `px4:droneA` | ok | px4_droneA.params |

## Gates

4 passed, 0 failed.

```
[PASS] must_equal       droneA:UXRCE_DDS_CFG expected=1000 actual=1000
[PASS] must_equal       droneA:UXRCE_DDS_PRT expected=8888 actual=8888
[PASS] must_equal       droneA:UXRCE_DDS_KEY expected=1 actual=1
[PASS] must_equal       droneA:UXRCE_DDS_DOM_ID expected=0 actual=0
```

## Rollback

- **droneA**: `rollback_console_droneA.txt`, `rollback_droneA.py`

Rollback restores the managed parameters in strict reverse of the apply order in `lab_config/expected_state.yaml`, to the values captured here.
