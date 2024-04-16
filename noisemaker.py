import time

# Delay for 1000 milliseconds (1 second)
time.sleep(0.001)       # use this to delay for a certain amount of time

def activate_buzzer():
    # Create a MAVLink message to trigger the buzzer
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target_system, target_component
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY,  # command
        0,  # confirmation
        0,  # param1: Relay number
        1,  # param2: Value: 1 for activate, 0 for deactivate
        0, 0, 0, 0, 0  # param3, param4, param5, param6, param7 (not used)
    )

    # Send the command to the vehicle
    vehicle.send_mavlink(msg)
