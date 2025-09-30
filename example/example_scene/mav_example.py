#! /usr/bin/env python3
import time
import sys
# Add pip'd libraries to path
sys.path.append('/localpkgs')
from pymavlink import mavutil



#
# Connect
#
def do_connect():
    # connect_string = "udpin:0.0.0.0:14550"
    # connect_string = "udpin:127.0.0.1:14550"
    connect_string = "udpin:127.0.0.1:14551"
    # connect_string = "tcp:127.0.0.1:5760"
    print (f"Connect string: {connect_string}")
    master = None
    try:
        master = mavutil.mavlink_connection(connect_string, timeout=5)
    except Exception as e:
        # Handle the connection fail
        print(f"Timeout occurred: {e}")

    return master
    
#
# Wait for a heartbeat with the specified timeout
# Returns False if fails
#
def do_wait_heartbeat(master):
    # Set the timeout duration (in seconds)
    timeout_duration = 10  # Change this value to adjust the timeout duration
    try:
        # Wait for a heartbeat with the specified timeout
        master.wait_heartbeat(timeout=timeout_duration)
        print("Heartbeat from system (system %u component %u)" %
            (master.target_system, master.target_component))
    except Exception as e:
        # Handle the timeout exception
        print(f"Timeout occurred: {e}")
        return False
    return True

###
###
def do_arm(master):
    # 2 ways to arm!
    master.arducopter_arm()
    # master.mav.command_long_send(
    #     master.target_system, master.target_component,
    #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
    #     0, 1, 0, 0, 0, 0, 0, 0)
    msg= master.recv_match(type="COMMAND_ACK", blocking=True)
    """
    Looks like:
     COMMAND_ACK {command : 400, result : 0, progress : 0, 
     result_param2 : 0, target_system : 255, target_component : 0}
    Where result 0 means OK
    """
    print(msg)
    if msg.result==0:
        print('armed')

###
# Takeoff
###
def do_takeoff(master):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
        0, 0, 0, 0, 0, 0, 0, 50)
    msg= master.recv_match(type="COMMAND_ACK", blocking=True)
    print(msg)

###
# Fly to position
###
def do_goto(master):
    master.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b100111111000), 40, 0, -10, 0, 0, 0, 0, 0, 0, 1.57, 0
            ))
    while 1:
        msg = master.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)

###
# Loiter
###
"""
cmd=mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST
print('before')
"""
master = do_connect()
#do_wait_heartbeat(master)
if master is None:
    print("No connection")
else:
    do_wait_heartbeat(master)
    do_arm(master)
    do_takeoff(master)
    # master.close()

print("Fini")
