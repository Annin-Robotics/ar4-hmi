# AR4 Python API User Guide

## 1. Overview

The AR4 Python API provides two classes for controlling the AR4 system from Python:

- `ARRobot` controls the Teensy 4.1 robot motion controller.
- `ARNano` controls the Arduino Nano IO board.

Most users will use `ARRobot` for robot motion, calibration, position requests, Modbus, and raw JSON pass-through commands. Use `ARNano` when controlling the auxiliary IO board, such as servo gripper commands, 5 V inputs, and 5 V digital outputs.

## 2. Recommended Folder Structure

For a simple standalone Python project, place `ARrobot.py` inside an `ARrobots` folder in your project.

```text
my_project/
├── main.py
├── defaults.json              # optional, required only for update_params()
└── ARrobots/
    ├── __init__.py
    └── ARrobot.py
```

The empty `__init__.py` file tells Python that `ARrobots` is a package.

Import the API like this:

```python
from ARrobots.ARrobot import ARRobot, ARNano
```

For a very small one-file test project, you may also place `ARrobot.py` in the same folder as your test script:

```text
my_project/
├── test_robot.py
└── ARrobot.py
```

Then import it like this:

```python
from ARrobot import ARRobot, ARNano
```

The package-folder method is recommended because it keeps the API separate from your own project files.

## 3. Requirements

Install `pyserial` before using the API:

```bash
pip install pyserial
```

The API uses standard Python modules plus `pyserial`.

## 4. Basic Robot Connection Example

```python
from ARrobots.ARrobot import ARRobot

robot = ARRobot(port="COM4")
robot.connect()

print(robot.hello())
print(robot.get_position())

robot.disconnect()
```

On Windows, the Teensy may appear as `COM4`, `COM5`, or another COM port. Use the port assigned by Windows Device Manager or the AR4 HMI COM port selector.

## 5. Basic Nano IO Board Connection Example

```python
from ARrobots.ARrobot import ARNano

nano = ARNano(port="COM5")
nano.connect()

print(nano.hello())

nano.gripper_open()
nano.gripper_close()

nano.disconnect()
```

The Nano IO board normally uses a different COM port than the Teensy motion controller.

## 6. Return Value Conventions

The API returns different types depending on the command:

- Robot position and status commands usually return a Python dictionary parsed from JSON.
- Simple firmware commands may return a raw string such as `"Done"`, `"Timeout"`, `"Modbus Error"`, or `"PARAMETERS_UPDATED"`.
- Some commands return `None` when no response is received or when the command is intentionally queued, such as linear moves during spline mode.
- Low-level send commands usually do not return anything.

A typical robot position response is a dictionary similar to:

```python
{
    "msg": "robot_pos",
    "j": [0.0, 0.0, 0.0, 0.0, 90.0, 0.0],
    "pose": [315.8, 0.0, 445.7, 0.0, 135.0, 0.0],
    "speed_violation": 0
}
```

A returned error usually has:

```python
{
    "msg": "error",
    "data": "error message"
}
```

Always check returned values before assuming the command succeeded.

## 7. ARRobot Class

Use `ARRobot` for the Teensy 4.1 robot controller.

### Constructor

```python
robot = ARRobot(port="COM4", baudrate=115200, timeout=None)
```

Arguments:

- `port`: COM port for the Teensy controller.
- `baudrate`: serial baud rate. Default is `115200`.
- `timeout`: serial timeout. Default is `None`.

### Connection Functions

| Function | Return | Description |
|---|---:|---|
| `connect()` | `True` or `False` | Opens the serial connection. |
| `disconnect()` | `None` | Closes the serial connection. |
| `is_connected()` | `True` or `False` | Returns whether the serial port is open. |

Example:

```python
robot = ARRobot(port="COM4")
if robot.connect():
    print("Robot connected")
```

### Low-Level Communication Functions

| Function | Return | Description |
|---|---:|---|
| `send(command)` | `None` | Sends a raw string command. A newline is added if missing. |
| `read_line()` | `str` | Reads one line from the robot controller. |
| `send_json(command_dict)` | `None` | Converts a dictionary to JSON and sends it. |
| `read_json()` | `dict` or `None` | Reads one response line and parses JSON if possible. |
| `read_response()` | `str` or `None` | Reads one raw response line. |
| `command(command_dict, timeout=120)` | `dict`, `str`, or `None` | Generic JSON pass-through command. Accepts a dictionary or JSON string. |
| `command_raw(command_text)` | `str` or `None` | Sends a raw command exactly as provided and reads one response. |

Most users should use the dedicated API methods or `command()`, not `send()` and `read_line()` directly.

Example using pass-through:

```python
response = robot.command({
    "cmd": "hello"
})

print(response)
```

Example using a JSON string:

```python
response = robot.command('{"cmd":"hello"}')
print(response)
```

### Information Functions

| Function | Return | Description |
|---|---:|---|
| `hello()` | `dict` | Requests controller information. |
| `get_position()` | `dict` or `None` | Returns the full robot position response. |
| `get_joints()` | `list` or `None` | Returns the current joint angles. |
| `get_pose()` | `list` or `None` | Returns the current Cartesian pose. |

Examples:

```python
print(robot.hello())

position = robot.get_position()
print(position["j"])
print(position["pose"])
```

```python
joints = robot.get_joints()
pose = robot.get_pose()

print("Joints:", joints)
print("Pose:", pose)
```

### Motion Functions

#### `move_joint_angles()`

```python
move_joint_angles(j1, j2, j3, j4, j5, j6, speed=25, acc=10, dec=10)
```

Moves the robot to specific joint angles.

Returns a robot position dictionary or an error dictionary.

Example:

```python
response = robot.move_joint_angles(
    0, 0, 0, 0, 90, 0,
    speed=25,
    acc=10,
    dec=10
)

print(response)
```

#### `move_joints()`

```python
move_joints(joints, speed=25, acc=10, dec=10)
```

Moves the robot using a list of joint values. The list must contain at least six values.

Returns a robot position dictionary or an error dictionary.

Example:

```python
joints = robot.get_joints()
joints[0] += 10

response = robot.move_joints(joints, speed=20)
print(response)
```

#### `move_cartesian()`

```python
move_cartesian(x, y, z, rx, ry, rz, speed=25, acc=10, dec=10, wrist="A")
```

Performs a joint-interpolated Cartesian move.

Returns a robot position dictionary or an error dictionary.

Example:

```python
response = robot.move_cartesian(
    x=315,
    y=0,
    z=450,
    rx=0,
    ry=135,
    rz=0,
    speed=25,
    wrist="A"
)

print(response)
```

#### `move_pose()`

```python
move_pose(pose, speed=25, acc=10, dec=10, wrist="A")
```

Moves to a Cartesian pose using a six-value pose list:

```python
[x, y, z, rx, ry, rz]
```

Returns a robot position dictionary or an error dictionary.

Example:

```python
pose = robot.get_pose()
pose[2] += 50

response = robot.move_pose(pose)
print(response)
```

#### `move_linear()`

```python
move_linear(x, y, z, rx, ry, rz, speed=25, acc=10, dec=10, rounding=0, wrist="A")
```

Moves the robot in a straight line to a Cartesian position.

Returns a robot position dictionary or an error dictionary. If spline mode is active, this function returns `None` because the move is queued for lookahead execution.

Example:

```python
response = robot.move_linear(
    315, 0, 450,
    0, 135, 0,
    speed=20,
    rounding=0
)

print(response)
```

#### `move_arc()`

```python
move_arc(mid, end, speed=25, acc=10, dec=10, ramp=50, wrist="A")
```

Moves through an arc. The current robot position is used as the start point.

Arguments:

- `mid`: six-value midpoint pose `[x, y, z, rx, ry, rz]`
- `end`: three-value endpoint `[x, y, z]`

Returns a robot position dictionary or an error dictionary.

Example:

```python
response = robot.move_arc(
    mid=[300, 50, 400, 0, 135, 0],
    end=[350, 0, 400],
    speed=20
)

print(response)
```

#### `move_circle()`

```python
move_circle(center, plane, orientation, speed=25, acc=10, dec=10, ramp=50, wrist="A")
```

Moves the robot in a circular path.

Arguments:

- `center`: `[x, y, z]`
- `plane`: `[x, y, z]`
- `orientation`: `[rx, ry, rz]`

Returns a robot position dictionary or an error dictionary.

Example:

```python
response = robot.move_circle(
    center=[315, 0, 400],
    plane=[315, 50, 400],
    orientation=[0, 135, 0],
    speed=20
)

print(response)
```

#### `stop()`

```python
stop()
```

Sends a stop command to the robot controller.

Return value: `None`.

Example:

```python
robot.stop()
```

## 8. Spline / Lookahead Motion

Spline mode allows multiple linear moves to be queued together for smoother path motion.

| Function | Return | Description |
|---|---:|---|
| `spline_start()` | `None` | Starts spline/lookahead mode. |
| `spline_end()` | `dict` or `None` | Ends spline/lookahead mode and returns the final robot position or error. |

Example:

```python
robot.spline_start()

robot.move_linear(300, 0, 400, 0, 135, 0, speed=25)
robot.move_linear(350, 0, 400, 0, 135, 0, speed=25)
robot.move_linear(350, 50, 400, 0, 135, 0, speed=25)

response = robot.spline_end()
print(response)
```

During spline mode, `move_linear()` returns `None` because the moves are being queued.

## 9. Calibration

```python
calibrate(stage1=None, stage2=None, offsets=None)
```

Calibrates robot axes.

Arguments:

- `stage1`: list of axes to calibrate in stage 1.
- `stage2`: list of axes to calibrate in stage 2.
- `offsets`: calibration offset values.

Each list may contain six to nine values. Missing J7-J9 values are filled with zero.

Default behavior calibrates J1-J6 in stage 1 only.

Returns a robot position dictionary, an error dictionary, or `None`.

Examples:

Calibrate all six main axes:

```python
response = robot.calibrate()
print(response)
```

Calibrate only J1:

```python
response = robot.calibrate(
    stage1=[1, 0, 0, 0, 0, 0]
)

print(response)
```

Use two-stage calibration:

```python
response = robot.calibrate(
    stage1=[1, 1, 1, 1, 1, 1],
    stage2=[0, 1, 1, 0, 0, 0],
    offsets=[0, 0, 0, 0, 0, 0]
)

print(response)
```

## 10. Wait Time

```python
wait_time(seconds)
```

Pauses the Python script for the specified number of seconds.

Return value: `None`.

Example:

```python
robot.wait_time(2)
```

This function uses Python `time.sleep()` and does not send a wait command to the controller.

## 11. Modbus Functions

The Modbus functions send JSON Modbus commands to the Teensy controller.

### Modbus Read Functions

| Function | Return | Description |
|---|---:|---|
| `modbus_read_coil(slave_id=1, address=0, count=1)` | `str` or `None` | Reads Modbus coil status. |
| `modbus_read_discrete_input(slave_id=1, address=0, count=1)` | `str` or `None` | Reads Modbus discrete input status. |
| `modbus_read_holding_register(slave_id=1, address=0, count=1)` | `str` or `None` | Reads holding register value. |
| `modbus_read_input_register(slave_id=1, address=0, count=1)` | `str` or `None` | Reads input register value. |

Examples:

```python
value = robot.modbus_read_coil(
    slave_id=7,
    address=3072,
    count=1
)

print(value)
```

```python
value = robot.modbus_read_holding_register(
    slave_id=7,
    address=1088,
    count=1
)

print(value)
```

### Modbus Write Functions

| Function | Return | Description |
|---|---:|---|
| `modbus_write_coil(slave_id=1, address=0, value=0)` | `str` or `None` | Writes a Modbus coil. |
| `modbus_write_register(slave_id=1, address=0, value=0)` | `str` or `None` | Writes a Modbus holding register. |

Examples:

```python
response = robot.modbus_write_coil(
    slave_id=7,
    address=3072,
    value=1
)

print(response)
```

```python
response = robot.modbus_write_register(
    slave_id=7,
    address=1088,
    value=55
)

print(response)
```

### Modbus Wait Functions

| Function | Return | Description |
|---|---:|---|
| `wait_modbus_coil(slave_id=1, address=0, expected=1, timeout=10)` | `str` or `None` | Waits for a coil to match the expected value. |
| `wait_modbus_input(slave_id=1, address=0, expected=1, timeout=10)` | `str` or `None` | Waits for a discrete input to match the expected value. |
| `wait_modbus_holding_register(slave_id=1, address=0, expected=0, timeout=10)` | `str` or `None` | Waits for a holding register to match the expected value. |

Expected firmware responses are typically:

- `"Done"`
- `"Timeout"`
- `"Modbus Error"`

Examples:

```python
response = robot.wait_modbus_coil(
    slave_id=7,
    address=3072,
    expected=1,
    timeout=10
)

print(response)
```

```python
response = robot.wait_modbus_input(
    slave_id=7,
    address=2048,
    expected=1,
    timeout=15
)

print(response)
```

```python
response = robot.wait_modbus_holding_register(
    slave_id=7,
    address=1088,
    expected=55,
    timeout=10
)

print(response)
```

## 12. Updating Robot Parameters

```python
update_params(config_file="defaults.json")
```

Loads robot parameters from a JSON configuration file and sends them to the controller.

Return value:

- `"PARAMETERS_UPDATED"` if successful.
- `"UPDATE FAILED: ..."` if the controller returns another response.
- `None` if no response is received.

The API searches for the configuration file in these locations:

1. The explicit path supplied by the user.
2. The same folder as `ARrobot.py`.
3. The parent folder of `ARrobot.py`.
4. The current working directory.

Example folder layout:

```text
my_project/
├── defaults.json
└── ARrobots/
    ├── __init__.py
    └── ARrobot.py
```

Example:

```python
response = robot.update_params("defaults.json")
print(response)
```

## 13. Generic JSON Pass-Through

The `command()` function is useful when the firmware supports a command that does not yet have a dedicated Python wrapper.

```python
response = robot.command({
    "cmd": "get_position"
})

print(response)
```

It also accepts JSON strings:

```python
response = robot.command('{"cmd":"get_position"}')
print(response)
```

Use this for testing new firmware commands without modifying the Python API.

## 14. Complete Robot Motion Example

```python
from ARrobots.ARrobot import ARRobot

robot = ARRobot(port="COM4")
robot.connect()

print("Hello:", robot.hello())

start_pose = robot.get_pose()
print("Start pose:", start_pose)

# Move up 50 mm
pose = start_pose.copy()
pose[2] += 50
print(robot.move_pose(pose, speed=25))

# Move back down
print(robot.move_pose(start_pose, speed=25))

# Move J1 by 10 degrees
joints = robot.get_joints()
joints[0] += 10
print(robot.move_joints(joints, speed=20))

robot.disconnect()
```

## 15. ARNano Class

Use `ARNano` for the Arduino Nano IO board.

### Constructor

```python
nano = ARNano(port="COM5", baudrate=9600, timeout=None)
```

Arguments:

- `port`: COM port for the Nano IO board.
- `baudrate`: serial baud rate. Default is `9600`.
- `timeout`: serial timeout. Default is `None`.

### Connection Functions

| Function | Return | Description |
|---|---:|---|
| `connect(port=None)` | `True` or `False` | Opens the Nano serial connection. |
| `disconnect()` | `None` | Closes the Nano serial connection. |
| `is_connected()` | `True` or `False` | Returns whether the Nano serial port is open. |

Example:

```python
nano = ARNano(port="COM5")
nano.connect()
```

### Low-Level Nano Functions

| Function | Return | Description |
|---|---:|---|
| `send_json(command_dict)` | `None` | Sends a JSON command dictionary to the Nano. |
| `read_response()` | `str` or `None` | Reads one raw response line. |
| `read_json()` | `dict` or `None` | Reads one response and parses JSON if possible. |
| `command(command_dict)` | `dict`, `str`, or `None` | Generic Nano JSON pass-through. Accepts a dictionary or JSON string. |

Example:

```python
response = nano.command({
    "cmd": "hello"
})

print(response)
```

### Nano Information Function

| Function | Return | Description |
|---|---:|---|
| `hello()` | `dict`, `str`, or `None` | Requests Nano board information. |

Example:

```python
print(nano.hello())
```

A typical response is:

```python
{
    "board": "AR4 Nano IO",
    "firmware": "1.6"
}
```

### Servo and Gripper Functions

| Function | Return | Description |
|---|---:|---|
| `servo_write(num, pos)` | `dict`, `str`, or `None` | Writes a servo position. |
| `gripper_open()` | `dict`, `str`, or `None` | Opens the AR4 servo gripper. |
| `gripper_close()` | `dict`, `str`, or `None` | Closes the AR4 servo gripper. |
| `gripper_detach()` | `dict`, `str`, or `None` | Disables gripper servo PWM output. |

Examples:

```python
nano.servo_write(0, 45)
nano.servo_write(0, 0)
```

```python
nano.gripper_open()
nano.gripper_close()
nano.gripper_detach()
```

### Nano IO Functions

| Function | Return | Description |
|---|---:|---|
| `input_read(pin)` | `dict`, `str`, or `None` | Reads a Nano input pin. |
| `set_output(pin, state)` | `dict`, `str`, or `None` | Sets a Nano output pin. |
| `wait_input(pin, state, timeout)` | `dict`, `str`, or `None` | Waits for an input pin to reach a target state. |
| `test_gripper_amps()` | `dict`, `str`, or `None` | Runs the gripper amperage test. |
| `stop()` | `dict`, `str`, or `None` | Sends stop command to the Nano. |

Examples:

```python
value = nano.input_read(2)
print(value)
```

```python
nano.set_output(8, 1)
nano.set_output(8, 0)
```

```python
response = nano.wait_input(
    pin=2,
    state=1,
    timeout=10
)

print(response)
```

## 16. Complete Nano IO Example

```python
from ARrobots.ARrobot import ARNano
import time

nano = ARNano(port="COM5")
nano.connect()

print("Nano:", nano.hello())

print("Open gripper")
nano.gripper_open()
time.sleep(1)

print("Close gripper")
nano.gripper_close()
time.sleep(1)

print("Detach gripper")
nano.gripper_detach()

print("Turn DO8 on")
nano.set_output(8, 1)
time.sleep(1)

print("Turn DO8 off")
nano.set_output(8, 0)

nano.disconnect()
```

## 17. Using ARRobot and ARNano Together

```python
from ARrobots.ARrobot import ARRobot, ARNano
import time

robot = ARRobot(port="COM4")
nano = ARNano(port="COM5")

robot.connect()
nano.connect()

print(robot.hello())
print(nano.hello())

# Move robot to a position
robot.move_cartesian(315, 0, 450, 0, 135, 0, speed=25)

# Open gripper
nano.gripper_open()
time.sleep(1)

# Close gripper
nano.gripper_close()

robot.disconnect()
nano.disconnect()
```

## 18. Error Handling Example

```python
response = robot.move_cartesian(315, 0, 450, 0, 135, 0)

if response is None:
    print("No response from robot")

elif isinstance(response, dict) and response.get("msg") == "error":
    print("Robot error:", response)

else:
    print("Move complete:", response)
```

## 19. Recommended Programming Pattern

A good script should:

1. Import the API.
2. Create the robot or Nano object.
3. Connect.
4. Send commands.
5. Check responses.
6. Disconnect when finished.

Example:

```python
from ARrobots.ARrobot import ARRobot

robot = ARRobot(port="COM4")

try:
    robot.connect()
    print(robot.hello())

    response = robot.move_cartesian(315, 0, 450, 0, 135, 0)

    if response is None:
        print("No response")
    elif isinstance(response, dict) and response.get("msg") == "error":
        print("Error:", response)
    else:
        print("Success:", response)

finally:
    robot.disconnect()
```

## 20. Function Reference Summary

### ARRobot

Connection:

- `connect()`
- `disconnect()`
- `is_connected()`

Low-level communication:

- `send(command)`
- `read_line()`
- `send_json(command_dict)`
- `read_json()`
- `read_response()`
- `command(command_dict, timeout=120)`
- `command_raw(command_text)`

Information:

- `hello()`
- `get_position()`
- `get_joints()`
- `get_pose()`

Motion:

- `move_joint_angles(j1, j2, j3, j4, j5, j6, speed=25, acc=10, dec=10)`
- `move_joints(joints, speed=25, acc=10, dec=10)`
- `move_cartesian(x, y, z, rx, ry, rz, speed=25, acc=10, dec=10, wrist="A")`
- `move_pose(pose, speed=25, acc=10, dec=10, wrist="A")`
- `move_linear(x, y, z, rx, ry, rz, speed=25, acc=10, dec=10, rounding=0, wrist="A")`
- `move_arc(mid, end, speed=25, acc=10, dec=10, ramp=50, wrist="A")`
- `move_circle(center, plane, orientation, speed=25, acc=10, dec=10, ramp=50, wrist="A")`
- `stop()`

Spline:

- `spline_start()`
- `spline_end()`

Calibration and configuration:

- `calibrate(stage1=None, stage2=None, offsets=None)`
- `update_params(config_file="defaults.json")`
- `wait_time(seconds)`

Modbus:

- `modbus_read_coil(slave_id=1, address=0, count=1)`
- `modbus_read_discrete_input(slave_id=1, address=0, count=1)`
- `modbus_read_holding_register(slave_id=1, address=0, count=1)`
- `modbus_read_input_register(slave_id=1, address=0, count=1)`
- `modbus_write_coil(slave_id=1, address=0, value=0)`
- `modbus_write_register(slave_id=1, address=0, value=0)`
- `wait_modbus_coil(slave_id=1, address=0, expected=1, timeout=10)`
- `wait_modbus_input(slave_id=1, address=0, expected=1, timeout=10)`
- `wait_modbus_holding_register(slave_id=1, address=0, expected=0, timeout=10)`

### ARNano

Connection:

- `connect(port=None)`
- `disconnect()`
- `is_connected()`

Low-level communication:

- `send_json(command_dict)`
- `read_response()`
- `read_json()`
- `command(command_dict)`

Information:

- `hello()`

Servo and gripper:

- `servo_write(num, pos)`
- `gripper_open()`
- `gripper_close()`
- `gripper_detach()`

IO:

- `input_read(pin)`
- `set_output(pin, state)`
- `wait_input(pin, state, timeout)`
- `test_gripper_amps()`
- `stop()`

## 21. Notes for Users

- The robot controller and Nano IO board are separate devices and usually use separate COM ports.
- Always make sure the robot is calibrated before running motion commands.
- Use low speeds when testing new scripts.
- Check return values for errors before continuing to the next command.
- Keep `command()` available as an escape hatch for new firmware commands that do not yet have dedicated Python functions.
- Do not run motion commands without verifying that the robot work area is clear.
