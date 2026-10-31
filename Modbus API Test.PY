from ARrobots.ARrobot import ARRobot

robot = ARRobot(port="COM4")
robot.connect()

print("Robot:", robot.hello())

SLAVE = 7

print("\n----------------------------------------")
print("1. Read Coil")
print("----------------------------------------")
print(robot.modbus_read_coil(
    slave_id=SLAVE,
    address=3072,
    count=1
))

print("\n----------------------------------------")
print("2. Read Discrete Input")
print("----------------------------------------")
print(robot.modbus_read_discrete_input(
    slave_id=SLAVE,
    address=2048,
    count=1
))

print("\n----------------------------------------")
print("3. Read Holding Register")
print("----------------------------------------")
print(robot.modbus_read_holding_register(
    slave_id=SLAVE,
    address=1088,
    count=1
))

print("\n----------------------------------------")
print("4. Write Coil OFF")
print("----------------------------------------")
print(robot.modbus_write_coil(
    slave_id=SLAVE,
    address=3072,
    value=0
))

print("\n----------------------------------------")
print("5. Write Coil ON")
print("----------------------------------------")
print(robot.modbus_write_coil(
    slave_id=SLAVE,
    address=3072,
    value=1
))

print("\n----------------------------------------")
print("6. Write Holding Register = 333")
print("----------------------------------------")
print(robot.modbus_write_register(
    slave_id=SLAVE,
    address=1088,
    value=333
))

print("\n----------------------------------------")
print("7. Write Holding Register = 444")
print("----------------------------------------")
print(robot.modbus_write_register(
    slave_id=SLAVE,
    address=1088,
    value=444
))

print("\n----------------------------------------")
print("8. Wait For Coil")
print("----------------------------------------")
print(robot.wait_modbus_coil(
    slave_id=SLAVE,
    address=3072,
    expected=1,
    timeout=20
))

print("\n----------------------------------------")
print("9. Wait For Holding Register")
print("----------------------------------------")
print(robot.wait_modbus_holding_register(
    slave_id=SLAVE,
    address=1088,
    expected=444,
    timeout=20
))

robot.disconnect()

print("\nDone")