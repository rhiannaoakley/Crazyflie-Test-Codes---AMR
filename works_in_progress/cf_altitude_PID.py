#================================================================
# Thia code is a work in progress to control a Crazyflie drone
# using OptiTrack motion capture data. The code updates and writes to the 
# cflib commander.send_setpoint() function to control the drone's altitude 
# using a PID controller. I does not yet control the pitch, roll, or yaw rates,
# and thes experiences a lot of drift.
#================================================================

import time
import threading
import logging
import math
import sys  # Import sys for NatNetClient example
import socket
# import DataDescriptions
import numpy as np
import keyboard
import struct
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# --- NatNet Client Imports ---
# Add the path to the folder containing NatNetClient.py
natnet_client_folder = r'.\\NatNetSDK\\Samples\\PythonClient'  # Replace with the actual folder name
sys.path.insert(0, natnet_client_folder)
from NatNetClient import NatNetClient


# Configure logging
logging.basicConfig(level=logging.ERROR)
FILE_NAME = "LOGGING_TEST.TXT"


iteration = 0


# --- NatNet SDK Configuration ---
NATNET_IP = '127.0.0.1'  # Replace with your OptiTrack server IP if different
SERVER_IP = '127.0.0.1'  # Add server IP
MULTICAST_ADDRESS = '239.255.42.99'  # Multicast address
MARKER_SET_NAME = 'RigidBody'  # Replace with the name of your rigid body in Motive 

# --- Crazyflie Configuration ---
URI = 'radio://0/80/2M/E7E7E7E7E7'  # Replace with your Crazyflie URI
HOVER_HEIGHT_METERS = 0.7 #meters
TAKEOFF_THRUST = 20000  #  value ranging from 10001 (next to no power) to 60000 (full power)
HOVER_THRUST = 35000  

# --- Global Variables ---
cf_position = [None, None, None]
cf_connected = False
cf_control_enabled = False
cf = None
natnet_client = None
rigid_body_names = ["RigidBody","CrazyFlie"]  # Dictionary to store {id: name} - changed from ["RigidBody","CrazyFlie"]
iteration = 0

def update(frame):
    global graph

    # updating the data
    x.append(iteration)
    y.append(cf_position[2])

    # creating a new graph or updating the graph
    graph.set_xdata(x)
    graph.set_ydata(y)
    plt.xlim(x[0], x[-1])

# --- NatNet Data Handling ---
def receive_rigid_body_frame(id, position, rotation):
    global cf_position, rigid_body_names
    # print(f"Received frame for Rigid Body ID: {id-1}")
    if id - 1 <= len(rigid_body_names):
        model_name = rigid_body_names[id - 1]
        if model_name == MARKER_SET_NAME:
            # Motive Y-axis is up, Crazyflie Z-axis is up
            cf_position = [position[0], -position[2], position[1]]
    else:
        print(f"  Warning: Rigid body ID {id - 1} not found in name mapping.")

def process_data_descriptions(natnet_client):
    """
    Function to request and process data descriptions to get rigid body names.
    """
    global rigid_body_names
    natnet_client.send_request(
        natnet_client.command_socket,
        natnet_client.NAT_REQUEST_MODELDEF,
        "",
        (natnet_client.server_ip_address, natnet_client.command_port),
    )
    time.sleep(0.1)  # Give time for response (adjust as needed)

    try:
        data, addr = natnet_client.command_socket.recvfrom(
            65535
        )  # Adjust buffer size if needed
    except socket.timeout:
        print("Error: Timeout waiting for data description packet.")
        return
    except Exception as e:
        print(f"Error receiving data description packet: {e}")
        return

    message_id = get_message_id(data)
    if message_id == natnet_client.NAT_MODELDEF:
        offset = 2  # Start after the message ID
        num_data_descriptions = struct.unpack('<I', data[offset:offset + 4])[0]
        offset += 4

        print(f"Number of data descriptions: {num_data_descriptions}")

        for i in range(num_data_descriptions):
            # Parse descriptor
            desc_type = struct.unpack('<I', data[offset:offset + 4])[0]
            offset += 4

            if desc_type == 0:  # Type 0: Rigid Body
                offset, rigid_body_name, rigid_body_id = DataDescriptions.unpack_rigid_body_description(
                    data, offset
                )
                rigid_body_names[rigid_body_id] = rigid_body_name.decode(
                    'utf-8'
                )  # Store the name
                print(
                    f"  Rigid Body Name: {rigid_body_names[rigid_body_id]}, ID: {rigid_body_id}"
                )
            elif desc_type == 1:  # Skeleton
                offset = DataDescriptions.unpack_skeleton_description(data, offset)
            elif desc_type == 2:  # Marker Set
                offset = DataDescriptions.unpack_marker_set_description(data, offset)
            else:
                print(f"  Unknown data description type: {desc_type}")
    else:
        print(
            f"Error: Expected NAT_MODELDEF (ID={natnet_client.NAT_MODELDEF}), got message ID {message_id}"
        )
        return
    print("Rigid Body Names:", rigid_body_names)  # prints the rigid body names


def get_message_id(data):
    message_id = int.from_bytes(data[0:2], byteorder='little', signed=True)
    return message_id


def receive_new_frame(data_dict):
    order_list = [
        "frameNumber",
        "markerSetCount",
        "unlabeledMarkersCount",
        "rigidBodyCount",
        "skeletonCount",
        "labeledMarkerCount",
        "timecode",
        "timecodeSub",
        "timestamp",
        "isRecording",
        "trackedModelsChanged",
    ]
    dump_args = False
    if dump_args == True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "="
            if key in data_dict:
                out_string += data_dict[key] + " "
            out_string += "/"
        print(out_string)



# --- Crazyflie Control ---
def connected(scf):
    global cf_connected, cf_control_enabled, cf
    cf = scf.cf
    print(f"Connected to {scf.cf.link_uri}")
    cf_connected = True
    cf_control_enabled = True



def control_crazyflie(hover_duration=5):  # Add hover_duration parameter, default is 10 seconds
    global cf, cf_position, cf_connected, cf_control_enabled, natnet_client, iteration

    if not cf_connected:
        print("Crazyflie not connected. Aborting control.")
        return

    # --- Control Gains (Tune these values!) ---
    # KP_X = 0.1  # Proportional gain for X-axis control
    # KP_Y = 0.1  # Proportional gain for Y-axis control
    KP_Z = 100 # Proportional gain for Z-axis (altitude) control  - was 0.5
    KI_Z = 15 # Integral gain
    KD_Z = 1 #DERIVATIVE GAIN
    # KP_ROLL = 0.1
    # KP_PITCH = 0.1
    # KP_YAWRATE = 0.1 #proportional gain for yawrate

    # --- Target Position ---
    HOVER_AT_CURRENT_XY = True  # Set to False to use TARGET_X and TARGET_Y
    TARGET_X = 0.0  # Adjust as needed if HOVER_AT_CURRENT_XY is False
    TARGET_Y = 0.0  # Adjust as needed if HOVER_AT_CURRENT_XY is False
    TARGET_Z = HOVER_HEIGHT_METERS

    # --- Internal Variables to Store Initial XY Position ---
    initial_x = None
    initial_y = None
    # initial_yaw = 0.0 #store initial yaw
    # target_yaw = 0.0 #set target yaw
    previous_position = [0, 0, 0]  # Store the previous position

    crash_detected = False  # added crash detection
    # CRASH_VELOCITY_THRESHOLD = 2.0  # m/s - Tune this!
    CRASH_POSITION_DEVIATION_THRESHOLD = 1  # METERS - Tune this!
    # CRASH_ROTATION_RATE_THRESHOLD = 5  # radians/second - Tune this!
    takeoff_complete = False

    thrust = []
    starting_thrust = 40000 #for troubleshooting
    takeoff_complete = True    
    thrust.append(starting_thrust)
    I = starting_thrust 

    start_time = time.time()
    print("Initializing thrust...")
    cf.commander.send_setpoint(0, 0, 0, 0)  # Send zero thrust
    time.sleep(0.1)  # Give it a short time to initialize
    cf.commander.send_setpoint(0, 0, 0, starting_thrust) 
    print("Taking off...")


    while takeoff_complete:
        try:
            
            if all(p is not None for p in cf_position):
                current_x, current_y, current_z = cf_position
                current_yaw = 0 #get yaw from optitrack if needed

            # Set initial XY target and yaw based on the first received position
            if (
                HOVER_AT_CURRENT_XY
                and initial_x is None
                and initial_y is None
            ):
                initial_x = current_x
                initial_y = current_y
                initial_yaw = current_yaw #set initial yaw
                print(
                    f"Setting initial hover XY target: X={initial_x:.2f}, Y={initial_y:.2f}"
                )

            # Determine the target XY
            target_x = (
                initial_x if HOVER_AT_CURRENT_XY and initial_x is not None else TARGET_X
            )
            target_y = (
                initial_y if HOVER_AT_CURRENT_XY and initial_y is not None else TARGET_Y
            )
            # target_yaw = initial_yaw

            # Calculate errors  
            error_z = TARGET_Z - cf_position[2]
    

            I = I + KI_Z*(error_z)

            thrust_adjustment = KP_Z*error_z + I + KD_Z*(cf_position[2] - previous_position[2]) # Adjust thrust based on vertical error
            # Limit thrust
            thrust.append(min(60000, thrust_adjustment))  # These values need tuning Fix

            cf.commander.send_setpoint(
                0, 0, 0, int(thrust[-1])
            )  # Send the setpoint
            time.sleep(0.01)

            #     # --- Crash Detection ---
            # if previous_position[0] >= target_x + CRASH_POSITION_DEVIATION_THRESHOLD or \
            #         previous_position[0] <= target_x - CRASH_POSITION_DEVIATION_THRESHOLD or \
            #         previous_position[1] >= target_y + CRASH_POSITION_DEVIATION_THRESHOLD or \
            #         previous_position[1] <= target_y - CRASH_POSITION_DEVIATION_THRESHOLD or \
            #         previous_position[3] >= TARGET_Z + CRASH_POSITION_DEVIATION_THRESHOLD or \
            #         previous_position[3] <= TARGET_Z - CRASH_POSITION_DEVIATION_THRESHOLD:
            #     crash_detected = True
            #     print("Crash detected: Position deviation too high.")

            #  ouse_rror_z = error_z
            previous_position = [current_x, current_y, current_z]
            print(cf_position)
            print(error_z)
            iteration = iteration + 1

            with open(FILE_NAME, "a") as file:
                line = f"THRUST: {thrust[-1]}, POSITION: {cf_position}, TIME: {(time.time() - start_time)}" 
                file.writelines(line +'\n')


            # if crash_detected:
            #     print("Crash detected. Emergency Landing...")
            
            #     ramp_down = []
            #     ramp_down.append(thrust[-1])
            #     while True:
            #         ramp_down.append(0.75*ramp_down[-1])    # Tune Gain
            #         cf.commander.send_setpoint(0, 0, 0, ramp_down[-1])  
            #         time.sleep(2.0)
            #         if cf_position[2] <= 50:  # Check if the Crazyflie is close to ground
            #             break

                # cf.commander.send_stop_setpoint()
                # cf = None  # Reset cf object after landing
                # if natnet_client is not None:
                #     natnet_client.shutdown()
                # sys.exit(1)

        except Exception as e:
            print(f"Error during control: {e}")
            
        except keyboard.read_key() =='l':
            print("Landing...")
            ramp_down = []
            ramp_down.append(thrust[-1])

            if cf_position[2] >= 0.05:
                ramp_down.append(0.75*ramp_down[-1])    # Tune Gain
                cf.commander.send_setpoint(0, 0, 0, int(ramp_down[-1]))  
                time.sleep(2.0)

            if cf_position[2]<=0.05:
                cf.commander.send_stop_setpoint()
                cf = None  # Reset cf object after landing
                takeoff_complete = False
                if natnet_client is not None:
                    natnet_client.shutdown()  # Make sure to shutdown the NatNet client

# --- Main Execution ---
if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    troubleshoot = []
    try:
        # Initialize NatNet client
        natnet_client = NatNetClient()
        natnet_client.set_client_address(NATNET_IP)
        natnet_client.set_server_address(SERVER_IP)
        natnet_client.set_use_multicast(True) # Assuming you're using multicast

        # Set data handler
        natnet_client.new_frame_listener = receive_new_frame
        natnet_client.rigid_body_listener = receive_rigid_body_frame

        # Start NatNet client
        is_running = natnet_client.run('d')
        if not is_running:
            print("ERROR: Could not start NatNet streaming client.")
            sys.exit(1)
        else:
            print("NatNet streaming client started.")

        process_data_descriptions(natnet_client)  # Get the rigid body names!

        # troubleshoot.append(cf_position)
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            if scf.cf is not None:
                connected(scf)
                time.sleep(5)

                control_thread = threading.Thread(target=control_crazyflie)
                control_thread.daemon = True
                control_thread.start()

                while cf_connected and cf_control_enabled:
                    time.sleep(0.1)

                if control_thread.is_alive():
                    control_thread.join()

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Exiting main.")
        if natnet_client is not None:
            natnet_client.shutdown()  # Make sure to shutdown the NatNet client

