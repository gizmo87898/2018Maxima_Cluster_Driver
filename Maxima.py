import can
import threading
import time
import random
import socket
import struct


send_lock = threading.Lock()

gear = 64  # 
speed = 0 # mp/h
lights = 0 # dahs lights
outside_temp = 72 # Farenheight - INACCURATE, FIX

#TPMS
fl_pressure = 31
fr_pressure = 32
rr_pressure = 14
rl_pressure = 31

#Cruise Control
cc_active = False
cc_speed = 50 # Set CC Speed
cc_speed_set = True # Is the speed set? (Yellow icon if not)

# Door status
fl_open = False
fr_open = False
rl_open = False
rr_open = False
trunk_open = False

# Power Steering
ps_light = False

# LDA, BSM, AEB
aeb_enabled = True
aeb_light = False
lda_enabled = True
bsm_enabled = True
lda_active = False
bsm_active = False

# Sonar Sensors
fl_sensor_dist = 0x4
fc_sensor_dist = 0x2
fr_sensor_dist = 0x4
rl_sensor_dist = 0x4
rc_sensor_dist = 0x2
rr_sensor_dist = 0x4

# Engine
mil = False
rpm = 750  # RPM 
engtemp = 100

# TC, ABS, Brake
brake_light = False
tc_status = 0x0 # 0x0 = On & Inactive, 0x1 = On & Active (Blinking), 0x2 = Off
abs_light = False

# Lights
parking_lights = True
fog_lights = False
left_directional = False
right_directional = False
high_beam = False

# Define the messages for 20ms interval
messages_20ms = [
    # Index
    # 0 RPM
    (0x180, [0, 0, 0, 0, 0, 0, 0, 0]),

]

# Define the messages for 100ms interval
messages_100ms = [
    # Index

    # 0 Airbag

    # 1 Speed

    # 2 Cruise Control, Blindspot/LDA/AEB (2b1)
    # start of 2b1
    # first 3:
    # 000 = no cc
    # 001 = MPH icon
    # 011 = green icon
    # 010 = grey icon
    # 100 = blinking grey
    # 111 = orange icon
    # Last 4:
    # 0001 = set speed icon instead of cc icon
    # 0010 = shows mph icon
    # 0100 = one bar of distance
    # 1000 = two bars
    # 1100 = three bars, cc icon
    # second byte 
    # 0b00000100 = blinking blind spot alert
    # 0b00001000 = ^
    # 0b00010000 = LDA blinking alert
    # 0b00100000 = ^
    # 0b01000000 = grey car icon on cruise control side
    # 0b10000000 = blinking orange car icon
    # 0b11110000 = AEB Alert
    # third byte
    # 00000001 = AEB light (1 = on)
    # 00000010 = AEB light slow blink
    # 00000100 = grey blindspot icon
    # 00001000 = grey blindspot icon slow blink
    # 00010000 = grey LDA icon
    # 00100000 = grey LDA icon slow blink
    # sixth byte
    # 00100000 = continuous beep
    # 01000000 = fast short beeps
    # 10000000 = slower short beeps
    # 11000000 = fast longer beeps
    # 11100000 = two short beeps
    # end of 2b1

    # 3 Tire Pressures (385)
    # start of 385
    # second byte (makes tires yellow)
    # 0b00000001 = rear left low
    # 0b00000010 = rear right low
    # 0b00000100 = front left low
    # 0b00001000 = front right low
    # third byte (front right)
    # fourth byte (front left)
    # fifth byte (rear right)
    # sixth byte (rear left)
    # end of 385

    # 4 Gear Indicator

    # 5 ABS, TC, Brake

    # 6 Power Steering

    # 7 TPMS Light, Oil Pressure

    # 8 Outside Temp

    # 9 Cruise Control, Engine Temp, MIL

    # 10 Outside Light Status, Door Status
    # 0x60d
    # first byte
    # 0b00000100 = parking lights icon
    # 0b00001000 = driver door open
    # 0b00010000 = passenger door open
    # 0b00100000 = driver rear door open
    # 0b01000000 = pass. rear door open
    # 0b10000000 = trunk open
    # second byte
    # 0b00001000 = tire pressure low alert
    # 0b00010000 = ^
    # 0b00
    # /0x60d

    # 11 Errors (351)
    # start of 351
    # 6th byte
    # 00000001 = "push brake and start switch to drive"
    # 00000011 = "key id incorrect"
    # 00000100 = "rotate the steering and push the start switch"
    # 00000101 = "shift to park" (red)
    # 00000110 = "power turned off to save the battery"
    # 00000111 = "key battery low"
    # 00001001 = "power will turn off to save the battery"
    # 00001010 = "push ignition to OFF"
    # 00001110 = "put key near start switch"
    # 00001000 = "no key detected"
    # 00010000 = continuous fast beeping
    # 00100000 = two short beeps continuous
    # 01000000 = "key system error"
    # 10000000 = "key registration complete"
    # 7th byte is beeping i didnt bother decoding it
    # end of 351

    # 12 Sport Mode
    # start of 5b0
    # first byte
    # 00000001 = sport mode active
    # end of 5b0

    # 13 Parking Sensors/ SONAR (57a)
    # start of 57a
    # first byte (sonar message)
    # 0b10000000 = sonar shows on cluster
    # 0b10110000 = parking sensor error
    # second byte (front + back sonar distance)
    # SENSOR VALUES
    # 0x1 = grey (inactive?)
    # 0x2 = full green
    # 0x4 = one grey bar (green)
    # 0x6 = two grey bars (yellow)
    # 0x8 = three grey bars (red)
    # third byte (rear left + right side sensors)
    # see sensor values
    # fourth byte (front left + right side sensors)
    # see sensor values
    # sixth bit (right side distance reading)
    # end of 57a

    (0x2a, [0, 0, 0, 0, 0, 0, 0, 0]), # airbag
    (0x284, [0, 0, 0, 0, 0, 0, 0, 0]), # speed
    (0x2b1, [0, 0, 0, 0, 0, 0, 0, 0]), # ordered: CC, blindspot/LDA/AEB alert, blindspot/lda/AEB light, null, null, AEB beeping, null, null
    (0x385, [0, 0b00000000, fr_pressure*4, fl_pressure*4, rr_pressure*4, rl_pressure*4, 0xff, 0]), # tire pressures bytes: null, makes it yellow, FR, FL, RR, RL, sensor status, avg front/ rear
    (0x421, [gear, 0, 0, 0, 0, 0, 0, 0]), # gear indicator
    (0x354, [0, 0, 0, 0, 0, 0, 0, 0]), # abs, tc, brake
    (0x5e4, [0, 0, 0, 0, 0, 0, 0, 0]), # power steering
    (0x358, [0, 0, 0, 0, 0, 0, 0, 0]), # tpms light, oil press.
    (0x54c, [0, 0, 0, 0, 0, 0, 0, 0]), # outside temp
    (0x551, [0, 0, 0, 0, 0, 0, 0, 0]), # cruise control, eng temp, mil
    (0x60d, [0, 0, 0, 0, 0, 0, 0, 0]), # lights, door warnings
    (0x351, [0, 0,0,0,0,0b00000000,0b00001000, 0]), # errors
    (0x5b0, [0b00000000, 0,0,0,0,0b00000000,0b00000000, 0]), # sport mode
    (0x57a, [0b10000000,0x22 ,0x44,0x44,0,0b00000000,0b00000000, 0]), # sonar
    

]
def send_messages_20ms(bus):
    while True:
        for message_id, data in messages_20ms:
            # Wait for 1ms before sending the message
            time.sleep(0.001)

            # Acquire the lock before sending a message
            send_lock.acquire()

            #Send message
            message = can.Message(arbitration_id=message_id, data=data, is_extended_id=False)
            bus.send(message, timeout=1)

            #print("Sent message (20ms):", message)

            # Release the lock after sending the message
            send_lock.release()

        # Wait for 20ms
        time.sleep(0.02)


def send_messages_100ms(bus):

    while True:
        for message_id, data in messages_100ms:
            # Wait for 1ms before sending the message
            time.sleep(0.001)

            # Acquire the lock before sending a message
            send_lock.acquire()

            

            # CC, AEB, LDA

            messages_100ms[3][1][2] = int(fr_pressure*4)
            messages_100ms[3][1][3] = int(fl_pressure*4)
            messages_100ms[3][1][4] = int(rr_pressure*4)
            messages_100ms[3][1][5] = int(rl_pressure*4)

            # Lights
            messages_100ms[5][1][4] = 0b00000000 #tracctrl_blink, tc_off, tc+tc_off_solid, n, abs, n, brake, n
            if is_bit_set(lights,2):
                messages_100ms[5][1][4] += 2 #(brake)
            if is_bit_set(lights,10):
                messages_100ms[5][1][4] += 8 #(abs)
            if is_bit_set(lights,4):
                messages_100ms[5][1][4] += 128 #(tracctrl_blink)


            
            # Outside Temp
            # 170 = 112f
            # 128 = 74f
            # 96 = 45f
            messages_100ms[8][1][6] = int(outside_temp*1.75)

            # Engine Temp
            messages_100ms[9][1][0] = max(int(engtemp*1.4), 0)

            # Sonar
            messages_100ms[13][1][1] = (fc_sensor_dist*16)+rc_sensor_dist
            messages_100ms[13][1][2] = rl_sensor_dist+(rr_sensor_dist*16)
            messages_100ms[13][1][3] = (fl_sensor_dist*16)+fr_sensor_dist
            print(messages_100ms[13][1][1])

            # Send message
            message = can.Message(arbitration_id=message_id, data=data, is_extended_id=False)
            bus.send(message, timeout=1)
            #print("Sent message (100ms):", message)

            # Release the lock after sending the message
            send_lock.release()

        # Wait for 100ms
        time.sleep(0.1)

def testing_5s(bus):
    while True:

        # Wait for 5s
        time.sleep(10000)


def is_bit_set(number, position):
    # Shift 1 to the position of the bit
    mask = 1 << position
    # Perform bitwise AND operation with the number and mask
    result = number & mask
    # Check if the result is non-zero (bit is set)
    return result != 0


def receive_messages(bus):
    while True:
        message = bus.recv()
        #print("Received message:", message)


def padhexa(s):
    return '0x' + s[2:].zfill(4)


def connect_to_game_socket():
    # Connect to the game socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 4444))

    # Receive data from the game and update variables accordingly
    while True:
        global rpm
        global speed
        global gear
        global engtemp
        global lights
        global messages_100ms
        global messages_20ms
        global fl_pressure
        global fr_pressure
        global rl_pressure
        global rr_pressure
        # Receive data.
        data, _ = sock.recvfrom(256)

        if not data:
            break  # Lost connection

        # Unpack the data.
        outgauge_pack = struct.unpack('I4sH2c7f2I7f16s16si', data)
        time_value = outgauge_pack[0]
        car = outgauge_pack[1]
        flags = outgauge_pack[2]
        gear = outgauge_pack[3]
        speed = outgauge_pack[5]*2.23694
        rpm = outgauge_pack[6]
        turbo = outgauge_pack[7]
        engtemp = outgauge_pack[8]
        fuel = outgauge_pack[9]
        oilpressure = outgauge_pack[10]
        oiltemp = outgauge_pack[11]
        dashlights = outgauge_pack[12]
        lights = outgauge_pack[13]
        throttle = outgauge_pack[14]
        brake = outgauge_pack[15]
        clutch = outgauge_pack[16]
        #print("Clutch: " + str(clutch))
        fl_pressure = outgauge_pack[17]
        fr_pressure = outgauge_pack[18]
        rl_pressure = outgauge_pack[19]
        rr_pressure = outgauge_pack[20]
        display1 = outgauge_pack[21]
        display2 = outgauge_pack[22]
        #print("FL: " + str(fl_pressure))
        #print("FR: " + str(fr_pressure))
        #print("RL: " + str(rl_pressure))
        #print("RR: " + str(rr_pressure))


        # Speed
        value = padhexa(hex(int(speed * 158)))
        messages_100ms[1][1][4] = int(value[2:4], 16)
        messages_100ms[1][1][5] = int(value[4:6], 16)

        value = padhexa(hex(int(max(min(rpm, 8000),0)) * 8))
        messages_20ms[0][1][0] = int(value[2:4], 16)
        messages_20ms[0][1][1] = int(value[4:6], 16)
        
        # gear
        match gear:
            case 'p':
                messages_100ms[4][1][0] = 8 # Park
            case b'\x00':
                messages_100ms[4][1][0] = 16 # Reverse
            case b'\x01':
                messages_100ms[4][1][0] = 24 # Neutral
            case "d":
                messages_100ms[4][1][0] = 32 # Drive
            case "ds":
                messages_100ms[4][1][0] = 40 # Drive Sport
            case "l":
                messages_100ms[4][1][0] = 48 # Low
            case b'\x02':
                messages_100ms[4][1][0] = 64 # 1st gear
            case b'\x03':
                messages_100ms[4][1][0] = 72 # 2nd gear
            case b'\x04':
                messages_100ms[4][1][0] = 80 # 3rd gear
            case b'\x05':
                messages_100ms[4][1][0] = 88 # 4th gear
            case b'\x06':
                messages_100ms[4][1][0] = 96 # 5th gear
            case b'\x07':
                messages_100ms[4][1][0] = 104 # 6th gear
            case b'\x08':
                messages_100ms[4][1][0] = 112 # 7th gear
            case b'\x09':
                messages_100ms[4][1][0] = 120 # 8th gear (works on cluster, not with beamng)

        #print(fr_pressure)
    # Close the socket connection
    sock.close()



# Create a CAN bus object
bus = can.interface.Bus(channel='com12', bustype='seeedstudio', bitrate=500000)

# Create threads for sending and receiving messages
send_thread_20ms = threading.Thread(target=send_messages_20ms, args=(bus,))
send_thread_100ms = threading.Thread(target=send_messages_100ms, args=(bus,))
testing_thread_5s = threading.Thread(target=testing_5s, args=(bus,))
receive_thread = threading.Thread(target=receive_messages, args=(bus,))
game_socket_thread = threading.Thread(target=connect_to_game_socket)

# Start the sending, receiving, and game socket threads
send_thread_20ms.start()
send_thread_100ms.start()
testing_thread_5s.start()
receive_thread.start()
game_socket_thread.start()


# Wait for the threads to finish
send_thread_20ms.join()
send_thread_100ms.join()
testing_thread_5s.join()
receive_thread.join()
game_socket_thread.join()

# Shutdown the CAN bus
bus.shutdown()
