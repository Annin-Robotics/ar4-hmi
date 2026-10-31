import serial
import json
import time
from pathlib import Path
import threading
import logging

logger = logging.getLogger("ARrobots")


# Reply returned instead of raising when a command is sent with no robot connected, so the
# HMI can show "Robot not connected" rather than crashing a worker thread.
NOT_CONNECTED_RESPONSE = '{"msg":"error","code":"N"}'


class ARRobot:
    def __init__(self, port=None, baudrate=115200, timeout=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self.spline_mode = False
        # command_lock: one request/response exchange at a time
        # write_lock:   serialises raw writes so stop() can be sent while another
        #               thread is blocked waiting inside command()/queue_move()
        self.command_lock = threading.Lock()
        self.write_lock = threading.Lock()
        # Called with a list of 6 joint angles for every live {"msg":"tele"} message the
        # firmware sends while streaming. Runs on whichever thread is reading the port.
        self.telemetry_callback = None

    def connect(self):
        if self.is_connected():
            return True

        if self.port in (None, "", "None"):
            raise ValueError("No COM port selected")

        self.serial_connection = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout
        )

        return self.is_connected()

    def is_connected(self):
        return (
            self.serial_connection is not None
            and self.serial_connection.is_open
        )

    def disconnect(self):
        if self.is_connected():
            self.serial_connection.close()

    def send(self, command):
        if not self.is_connected():
            raise RuntimeError("Robot is not connected")

        if not command.endswith("\n"):
            command += "\n"

        with self.write_lock:
            self.serial_connection.write(command.encode())
            self.serial_connection.flush()

    def read_line(self):
        if not self.is_connected():
            raise RuntimeError("Robot is not connected")

        return self.serial_connection.readline().decode("utf-8").strip()

    def send_json(self, command_dict):
        command = json.dumps(command_dict)
        self.send(command)

    def read_json(self):
        line = self.read_line()

        if not line:
            return None

        try:
            return json.loads(line)
        except json.JSONDecodeError:
            return {
                "msg": "raw",
                "data": line
            } 

    def get_position(self):
        self.send_json({
            "cmd": "get_position"
        })

        return self._wait_for_message("robot_pos")

    def get_joints(self):
        position = self.get_position()

        if position is None:
            return None

        return position.get("j") 

    def get_pose(self):
        position = self.get_position()

        if position is None:
            return None

        return position.get("pose") 

    def hello(self):
        self.send_json({
            "cmd": "hello"
        })

        return self.read_json()

    def move_joint_angles(self, j1, j2, j3, j4, j5, j6, speed=25, acc=10, dec=10):
        """
        Move the robot to the specified joint angles.

        Parameters:
            j1-j6: Joint angles in degrees
            speed: Motion speed percentage
            acc: Acceleration percentage
            dec: Deceleration percentage

        Returns:
            Robot position response dictionary or error response.
        """
        self.send_json({
            "cmd": "move_joints",
            "j": [j1, j2, j3, j4, j5, j6, 0, 0, 0],
            "spd_type": "percent",
            "spd": speed,
            "acc": acc,
            "dec": dec
        })

        return self._wait_for_message(["robot_pos", "error"], timeout=120)
    
    def move_joints(self, joints, speed=25, acc=10, dec=10):
        """
        Move the robot to the specified joint angles using a list.

        Parameters:
            joints: List containing 6 to 9 joint values.
                    Only J1-J6 are currently used.
            speed: Motion speed percentage
            acc: Acceleration percentage
            dec: Deceleration percentage

        Returns:
            Robot position response dictionary or error response.

        Example:
            joints = robot.get_joints()
            joints[0] += 10
            robot.move_joints(joints)
        """
        if len(joints) < 6:
            raise ValueError("move_joints requires at least 6 joint values")

        return self.move_joint_angles(
            joints[0],
            joints[1],
            joints[2],
            joints[3],
            joints[4],
            joints[5],
            speed=speed,
            acc=acc,
            dec=dec
        )

    def move_cartesian(self, x, y, z, rx, ry, rz, speed=25, acc=10, dec=10, wrist="A"):
        """
        Joint-interpolated move to a Cartesian position.

        Parameters:
            x, y, z: Position in mm
            rx, ry, rz: Orientation in degrees
            speed: Percent speed
            acc: Acceleration percentage
            dec: Deceleration percentage
        """
        self.send_json({
            "cmd": "move_j",
            "pose": [x, y, z, rx, ry, rz],
            "spd_type": "percent",
            "spd": speed,
            "acc": acc,
            "dec": dec,
            "w": wrist
        })

        return self._wait_for_message(["robot_pos", "error"], timeout=120)
    
    def move_linear(self, x, y, z, rx, ry, rz, speed=25, acc=10, dec=10, rounding=0, wrist="A"):
        """
        Move the robot in a straight line to a Cartesian position.

        Parameters:
            x, y, z: Target position in mm
            rx, ry, rz: Target orientation in degrees
            speed: Motion speed percentage
            acc: Acceleration percentage
            dec: Deceleration percentage
            rounding: Path blending radius

        Returns:
            Robot position response dictionary or error response.
        """
        self.send_json({
            "cmd": "move_l",
            "pose": [x, y, z, rx, ry, rz],
            "ext": [0, 0, 0],
            "spd_type": "percent",
            "spd": speed,
            "acc": acc,
            "dec": dec,
            "rounding": rounding,
            "w": wrist
        })

        if self.spline_mode:
            return None

        return self._wait_for_message(["robot_pos", "error"], timeout=120)

    def move_pose(self, pose, speed=25, acc=10, dec=10, wrist="A"):
        """
        Move the robot to a Cartesian pose using a pose list.

        Parameters:
            pose: [x, y, z, rx, ry, rz]
            speed: Motion speed percentage
            acc: Acceleration percentage
            dec: Deceleration percentage

        Returns:
            Robot position response dictionary or error response.

        Example:
            pose = robot.get_pose()
            pose[2] += 50
            robot.move_pose(pose)
        """
        return self.move_cartesian(
            pose[0],
            pose[1],
            pose[2],
            pose[3],
            pose[4],
            pose[5],
            speed=speed,
            acc=acc,
            dec=dec,
            wrist=wrist
        )

    def stop(self):
        if not self.is_connected():
            return
        """
        Soft stop. Sent WITHOUT taking command_lock so it can interrupt a program
        thread that is blocked waiting for an ack or for spline_end. The firmware
        halts immediately, drops all queued moves and sends one robot_pos reply,
        which is consumed by whichever call is currently waiting (or discarded as
        stale by the next command()).
        Only received mid-move while a stream (spline mode) is active.
        """
        self.send_json({
            "cmd": "stop"
        })

    def _wait_for_message(self, message_types, timeout=90):
        if isinstance(message_types, str):
            message_types = [message_types]

        start_time = time.time()

        while True:
            if timeout is not None and time.time() - start_time > timeout:
                return None

            response = self.read_json()

            if response is None:
                continue

            if response.get("msg") == "tele":
                if self.telemetry_callback is not None:
                    try:
                        self.telemetry_callback([float(v) for v in response.get("j", [])[:6]])
                    except Exception:
                        pass
                continue

            if response.get("msg") in message_types:
                return response

    def _normalize_axis_list(self, values, name):
        if values is None:
            values = [0, 0, 0, 0, 0, 0]

        if len(values) < 6 or len(values) > 9:
            raise ValueError(f"{name} must contain 6 to 9 values")

        return list(values) + [0] * (9 - len(values))

    def calibrate(self, stage1=None, stage2=None, offsets=None):
        """
        Calibrate robot axes.

        stage1: List of axes to calibrate in stage 1 (6-9 values)
        stage2: List of axes to calibrate in stage 2 (6-9 values)
        offsets: Calibration offsets (6-9 values)
        """
        if stage1 is None:
            stage1 = [1, 1, 1, 1, 1, 1]

        stage1 = self._normalize_axis_list(stage1, "stage1")

        if stage2 is None:
            stage2 = [0, 0, 0, 0, 0, 0]

        stage2 = self._normalize_axis_list(stage2, "stage2")

        if offsets is None:
            offsets = [0, 0, 0, 0, 0, 0]

        offsets = self._normalize_axis_list(offsets, "offsets")

        self.send_json({
            "cmd": "calibrate",
            "calibrate": stage1,
            "offsets": offsets
        })

        response = self._wait_for_message(["robot_pos", "error"], timeout=120)

        if response is None:
            return None

        if response.get("msg") == "error":
            return response

        if any(stage2):
            self.send_json({
                "cmd": "calibrate_stage2",
                "calibrate": stage2,
                "offsets": offsets
            })

            response = self._wait_for_message(["robot_pos", "error"], timeout=120)

        return response

    def wait_time(self, seconds):
        """
        Pause API script execution for the specified number of seconds.
        """
        time.sleep(seconds)
        
    def move_arc(self, mid, end, speed=25, acc=10, dec=10, ramp=50, wrist="A"):
        """
        Move the robot through an arc using the current position as the start point.

        Parameters:
            mid: [x, y, z, rx, ry, rz] arc midpoint pose
            end: [x, y, z] arc end point
            speed: Motion speed percentage
            acc: Acceleration percentage
            dec: Deceleration percentage
            ramp: Acceleration ramp percentage
            wrist: Wrist configuration, default "A"
        """

        if len(mid) != 6:
            raise ValueError("mid must contain 6 values: [x, y, z, rx, ry, rz]")

        if len(end) != 3:
            raise ValueError("end must contain 3 values: [x, y, z]")

        self.send_json({
            "cmd": "move_a",
            "mid": mid,
            "end": end,
            "spd_type": "percent",
            "spd": speed,
            "acc": acc,
            "dec": dec,
            "ramp": ramp,
            "w": wrist
        })

        return self._wait_for_message(["robot_pos", "error"], timeout=120)

    def move_circle(self, center, plane, orientation, speed=25, acc=10, dec=10, ramp=50, wrist="A"):
        """
        Move the robot in a circular path.

        Parameters:
            center: [x, y, z] circle center point
            plane: [x, y, z] point defining the circle plane
            orientation: [rx, ry, rz] tool orientation
        """

        if len(center) != 3:
            raise ValueError("center must contain 3 values: [x, y, z]")

        if len(plane) != 3:
            raise ValueError("plane must contain 3 values: [x, y, z]")

        if len(orientation) != 3:
            raise ValueError("orientation must contain 3 values: [rx, ry, rz]")

        self.send_json({
            "cmd": "move_c",
            "center": center,
            "plane": plane,
            "pose": orientation,
            "spd_type": "percent",
            "spd": speed,
            "acc": acc,
            "dec": dec,
            "ramp": ramp,
            "w": wrist
        })

        return self._wait_for_message(["robot_pos", "error"], timeout=120)

    def spline_start(self):
        """
        Start spline/lookahead mode.
        """
        self.spline_mode = True
        self.send_json({
            "cmd": "spline_start"
        })

    def spline_end(self):
        """
        End spline/lookahead mode and return final robot position.
        """
        self.send_json({
            "cmd": "spline_end"
        })

        self.spline_mode = False

        return self._wait_for_message(["robot_pos", "error"], timeout=120)  

    def read_response(self):
        """
        Read one raw response line from the robot controller.

        Returns:
            Raw string response, or None if no response was received.
        """
        response = self.read_line()

        if response == "":
            return None

        return response

    def modbus_read_holding_register(self, slave_id=1, address=0, count=1):
        self.send_json({
            "cmd": "modbus_read_holding_register",
            "slave_id": slave_id,
            "address": address,
            "count": count
        })

        return self.read_response()


    def modbus_read_coil(self, slave_id=1, address=0, count=1):
        self.send_json({
            "cmd": "modbus_read_coil",
            "slave_id": slave_id,
            "address": address,
            "count": count
        })

        return self.read_response()


    def modbus_read_discrete_input(self, slave_id=1, address=0, count=1):
        self.send_json({
            "cmd": "modbus_read_discrete_input",
            "slave_id": slave_id,
            "address": address,
            "count": count
        })

        return self.read_response()


    def modbus_read_input_register(self, slave_id=1, address=0, count=1):
        self.send_json({
            "cmd": "modbus_read_input_register",
            "slave_id": slave_id,
            "address": address,
            "count": count
        })

        return self.read_response()


    def modbus_write_coil(self, slave_id=1, address=0, value=0):
        self.send_json({
            "cmd": "modbus_write_coil",
            "slave_id": slave_id,
            "address": address,
            "value": value
        })

        return self.read_response()


    def modbus_write_register(self, slave_id=1, address=0, value=0):
        self.send_json({
            "cmd": "modbus_write_register",
            "slave_id": slave_id,
            "address": address,
            "value": value
        })

        return self.read_response()


    def wait_modbus_coil(self, slave_id=1, address=0, expected=1, timeout=10):
        self.send_json({
            "cmd": "wait_modbus_coil",
            "slave_id": slave_id,
            "address": address,
            "expected": expected,
            "timeout": timeout
        })

        return self.read_response()


    def wait_modbus_input(self, slave_id=1, address=0, expected=1, timeout=10):
        self.send_json({
            "cmd": "wait_modbus_input",
            "slave_id": slave_id,
            "address": address,
            "expected": expected,
            "timeout": timeout
        })

        return self.read_response()
    
    def wait_modbus_holding_register(self, slave_id=1, address=0, expected=0, timeout=10):
        self.send_json({
            "cmd": "wait_modbus_holding_register",
            "slave_id": slave_id,
            "address": address,
            "expected": expected,
            "timeout": timeout
        })

        return self.read_response()  

    def _build_update_params_payload(self, config):

        def get(key):
            return str(config[key]).replace(",", "")

        payload = [
            get("TFx"), get("TFy"), get("TFz"), get("TFrz"), get("TFry"), get("TFrx"),

            get("J1MotDir"), get("J2MotDir"), get("J3MotDir"), get("J4MotDir"), get("J5MotDir"), get("J6MotDir"),
            get("J7MotDir"), get("J8MotDir"), get("J9MotDir"),

            get("J1CalDir"), get("J2CalDir"), get("J3CalDir"), get("J4CalDir"), get("J5CalDir"), get("J6CalDir"),
            get("J7CalDir"), get("J8CalDir"), get("J9CalDir"),

            get("J1CalSwitch"), get("J2CalSwitch"), get("J3CalSwitch"), get("J4CalSwitch"), get("J5CalSwitch"), get("J6CalSwitch"),
            get("J7CalSwitch"), get("J8CalSwitch"), get("J9CalSwitch"),

            get("J1PosLim"), get("J1NegLim"), get("J2PosLim"), get("J2NegLim"), get("J3PosLim"), get("J3NegLim"),
            get("J4PosLim"), get("J4NegLim"), get("J5PosLim"), get("J5NegLim"), get("J6PosLim"), get("J6NegLim"),

            get("J1StepDeg"), get("J2StepDeg"), get("J3StepDeg"), get("J4StepDeg"), get("J5StepDeg"), get("J6StepDeg"),

            str(float(config["J1EncCPR"]) / float(config["J1DriveMS"])),
            str(float(config["J2EncCPR"]) / float(config["J2DriveMS"])),
            str(float(config["J3EncCPR"]) / float(config["J3DriveMS"])),
            str(float(config["J4EncCPR"]) / float(config["J4DriveMS"])),
            str(float(config["J5EncCPR"]) / float(config["J5DriveMS"])),
            str(float(config["J6EncCPR"]) / float(config["J6DriveMS"])),

            get("J1ΘDHpar"), get("J2ΘDHpar"), get("J3ΘDHpar"), get("J4ΘDHpar"), get("J5ΘDHpar"), get("J6ΘDHpar"),
            get("J1αDHpar"), get("J2αDHpar"), get("J3αDHpar"), get("J4αDHpar"), get("J5αDHpar"), get("J6αDHpar"),
            get("J1dDHpar"), get("J2dDHpar"), get("J3dDHpar"), get("J4dDHpar"), get("J5dDHpar"), get("J6dDHpar"),
            get("J1aDHpar"), get("J2aDHpar"), get("J3aDHpar"), get("J4aDHpar"), get("J5aDHpar"), get("J6aDHpar"),
        ]

        return payload

    def _find_config_file(self, config_file="defaults.json"):
        path = Path(config_file)

        if path.is_absolute() and path.exists():
            return path

        candidates = [
            Path(__file__).parent / config_file,
            Path(__file__).parent.parent / config_file,
            Path.cwd() / config_file,
        ]

        for candidate in candidates:
            if candidate.exists():
                return candidate

        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    def update_params(self, config_file="defaults.json"):
        """
        Load robot parameters from a defaults/config JSON file and send them
        to the robot controller.
        """

        config_path = self._find_config_file(config_file)

        with open(config_path, "r") as f:
            config = json.load(f)

        payload = self._build_update_params_payload(config)

        self.send_json({
            "cmd": "update_params",
            "params": payload
        })

        response = self.read_response()

        if response == "PARAMETERS_UPDATED":
            return response

        return f"UPDATE FAILED: {response}"

    def command_raw(self, command_text):
        """
        Send a raw legacy serial command exactly as provided.
        Used for legacy commands like S\n.
        """
        if not self.is_connected():
            return NOT_CONNECTED_RESPONSE

        if not command_text.endswith("\n"):
            command_text += "\n"

        with self.write_lock:
            self.serial_connection.write(command_text.encode())

        return self.read_response()

    # ------------------------------------------------------------------
    # Motion streaming helpers
    # ------------------------------------------------------------------
    def _handle_telemetry(self, line):
        """True if `line` was a live telemetry message (consumed, never a reply)."""
        if '"msg":"tele"' not in line:
            return False
        cb = self.telemetry_callback
        if cb is not None:
            try:
                j = json.loads(line).get("j", [])
                if len(j) >= 6:
                    cb([float(v) for v in j[:6]])
            except Exception as e:
                logger.debug(f"Telemetry handling failed: {e}")
        return True

    @staticmethod
    def _msg_type(line):
        try:
            return json.loads(line).get("msg")
        except (json.JSONDecodeError, TypeError, AttributeError):
            return None

    def _drain_pending(self):
        """
        Return (and remove) every complete line already waiting in the input
        buffer, without waiting for new data. These are unsolicited/stale lines:
        nothing should be outstanding when a new request is about to be sent.
        """
        lines = []
        while self.serial_connection.in_waiting > 0:
            raw = self.serial_connection.readline()
            if not raw:
                break
            line = raw.decode(errors="replace").strip()
            if line and not self._handle_telemetry(line):
                lines.append(line)
        if lines:
            logger.debug(f"Discarded stale robot lines: {lines}")
        return lines

    def queue_move(self, command_dict, timeout=120):
        if not self.is_connected():
            return NOT_CONNECTED_RESPONSE
        """
        Send a move to the firmware motion queue (spline mode must be active).

        Returns the raw response line:
          {"msg":"ack",...}       move accepted and queued (normal case)
          {"msg":"error",...}     a queued move faulted, or this move was rejected
          {"msg":"robot_pos",...} a stop was processed while waiting

        Blocks only while the firmware buffer is full, which gives natural
        back-pressure: Python never runs more than two moves ahead of the robot.
        """
        with self.command_lock:
            if isinstance(command_dict, str):
                command_dict = json.loads(command_dict)
            command_dict = dict(command_dict)
            command_dict["q"] = 1

            # An error that arrived since the last ack belongs to an earlier queued
            # move. Report it instead of sending another move.
            for line in self._drain_pending():
                if self._msg_type(line) == "error":
                    return line

            self.send_json(command_dict)

            start_time = time.time()
            while True:
                if time.time() - start_time > timeout:
                    raise TimeoutError(f"Timed out waiting for queue ack: {command_dict}")

                raw = self.serial_connection.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if not line or self._handle_telemetry(line):
                    continue
                if self._msg_type(line) in ("ack", "error", "robot_pos"):
                    return line
                logger.debug(f"Ignored while waiting for queue ack: {line}")
    
    def command(self, command_dict, timeout=120):
        if not self.is_connected():
            return NOT_CONNECTED_RESPONSE
        with self.command_lock:
            if isinstance(command_dict, str):
                command_dict = json.loads(command_dict)

            # Anything already waiting is not a reply to this command
            self._drain_pending()

            self.send_json(command_dict)

            start_time = time.time()

            while True:
                if time.time() - start_time > timeout:
                    raise TimeoutError(f"Timed out waiting for response to command: {command_dict}")

                response = self.serial_connection.readline()

                if not response:
                    continue

                response = response.decode(errors="replace").strip()

                if response and not self._handle_telemetry(response):
                    return response


class ARNano:

    def __init__(self, port=None, baudrate=9600, timeout=2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None

    def connect(self, port=None):

        if port is not None:
            self.port = port

        if not self.port:
            raise ValueError("No serial port specified")

        if self.serial_connection and self.serial_connection.is_open:
            return True

        self.serial_connection = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout
        )

        time.sleep(2)

        return True

    def disconnect(self):

        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

    def is_connected(self):
        return (
            self.serial_connection is not None
            and self.serial_connection.is_open
        )

    def send_json(self, command_dict):
        if not self.is_connected():
            raise RuntimeError("Nano not connected")

        command = json.dumps(command_dict) + "\n"
        self.serial_connection.write(command.encode("utf-8"))

    def read_response(self):
        if not self.is_connected():
            raise RuntimeError("Nano not connected")

        while True:
            response = self.serial_connection.readline().decode("utf-8").strip()
            if response:
                return response

    def read_json(self):
        response = self.read_response()

        try:
            return json.loads(response)
        except json.JSONDecodeError:
            return response

    def command(self, command_dict):

        if isinstance(command_dict, str):
            command_dict = json.loads(command_dict)

        self.send_json(command_dict)

        return self.read_json()

    def hello(self):
        return self.command({
            "cmd": "hello"
        })

    def servo_write(self, num, pos):
        return self.command({
            "cmd": "servo",
            "num": int(num),
            "pos": int(pos)
        })

    def input_read(self, pin):
        response = self.command({
            "cmd": "input_read",
            "pin": int(pin)
        })

        if response == "T":
            return True
        if response == "F":
            return False

        return response

    def set_output(self, pin, state):
        return self.command({
            "cmd": "set_output",
            "pin": int(pin),
            "state": int(bool(state))
        })

    def wait_input(self, pin, state, timeout):
        return self.command({
            "cmd": "wait_input",
            "pin": int(pin),
            "state": int(state),
            "timeout": int(timeout)
        })
  

    def test_gripper_amps(self):
        response = self.command({
            "cmd": "test_gripper_amps"
        })

        try:
            return float(response)
        except ValueError:
            return response

    def stop(self):
        return self.command({
            "cmd": "stop"
        }) 


    def gripper_open(self):
        """
        Open the servo gripper.
        """
        return self.servo_write(0, 45)


    def gripper_close(self):
        """
        Close the servo gripper.
        """
        return self.servo_write(0, 0)


    def gripper_detach(self):
        """
        Disable the gripper servo PWM output.
        """
        return self.command({
            "cmd": "gripper_detach"
        })                  
            
                   


