# PS4 Joy Teleop

How to drive Spot with the PS4 controller using the `spot_joy` package
(`src/spot_nav/spot_joy`).

## 1. Connect the controller

The teleop launch binds to a udev symlink `/dev/input/ps4_primary`, created by
`/etc/udev/rules.d/99-ps4-primary.rules` for one specific controller
(Bluetooth MAC `bc:c7:46:58:0e:fb`). Connect it on the **host**:

```bash
bluetoothctl connect BC:C7:46:58:0E:FB
```

If it has never been paired: put the controller in pairing mode by holding
**Share + PS** until the light bar flashes, then in `bluetoothctl` run
`scan on` → `pair` → `trust` → `connect`.

Verify the device exists before launching:

```bash
ls -l /dev/input/ps4_primary
```

> **Different controller?** Only the controller with the MAC above matches the
> udev rule. For another pad, either pass `joy_dev:=/dev/input/js0` to the
> launch file or add its MAC to the udev rule.

## 2. Launch teleop

ROS runs in the Docker container, so from inside the container:

```bash
ros2 launch spot_joy teleop.launch.py
```

This starts:

- `joy_linux_node` reading `/dev/input/ps4_primary` (auto-respawns after a
  disconnect; delay configurable via `joy_respawn_delay`, default 2 s)
- `spot_joy_teleop` translating `joy` messages into Spot commands

Launch arguments:

| Arg | Default | Purpose |
|---|---|---|
| `joy_dev` | `/dev/input/ps4_primary` | Joystick device path |
| `verbose` | `false` | Per-second logging of mode/speed state |
| `joy_respawn_delay` | `2.0` | Seconds before respawning joy node after disconnect |

## 3. Controls

| Input | Action |
|---|---|
| **Options** | Claim + power on Spot |
| **Square** | Stand (enters body-pose mode) |
| **Cross** | Sit (stops motion, suppresses input for 2 s) |
| **Triangle** | Walk mode |
| **Circle** | Toggle Auto Dog Mode (`/dog_mode/enable`) |
| **D-pad up/down** | Body height ±0.05 m (clamped to ±0.3 m) |
| **L1 (hold)** | Deadman for manual control — required to move |
| **R1 (hold)** | Deadman for auto mode (passes `/cmd_vel_intermediate` through to `/cmd_vel`) |
| **L2 (hold)** | Fast speed (1.0 m/s); otherwise slow (0.5 m/s) |

While holding L1:

- **Walk mode:** left stick = forward/strafe, right stick left/right = turn.
- **Stand mode** (after Square): left stick left/right = roll,
  right stick = pitch/yaw.

Releasing L1 immediately publishes zero velocity and a neutral body pose
(safety stop) — nothing moves unless a deadman is held.

## Typical session

1. Connect controller (`bluetoothctl connect ...`)
2. `ros2 launch spot_joy teleop.launch.py` (in the container)
3. **Options** — claim and power on
4. **Square** (stand) or **Triangle** (walk mode)
5. Hold **L1** and drive; hold **L2** as well for full speed

## Troubleshooting

- **No `/dev/input/ps4_primary`** — controller not connected, or it's a
  different pad than the one in the udev rule.
- **Robot won't move while driving** — the teleop node warns if motors aren't
  powered or Spot isn't standing; press **Options** then **Triangle** and
  confirm Spot is claimed/powered.
- **Speed stuck fast/slow** — `joy_linux` trigger axes rest at 0.0 until first
  touched, then rest at 1.0; the node handles this, but fully press and
  release L2 once after connecting if speed scaling seems off.
