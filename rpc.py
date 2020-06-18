#This is the python file for xbee base(PC)
import serial
import numpy as np
import matplotlib.pyplot as plt
import time as t
import paho.mqtt.client as paho

serdev = '/dev/ttyUSB0'

s = serial.Serial(serdev, 9600)

mqttc = paho.Client()


# Settings for connection from xbee base to host PC for later mqtt

host = "localhost"

port = 1883

# MQTT Callbacks

def on_connect(self, mosq, obj, rc):

    print("Connected rc: " + str(rc))


def on_message(mosq, obj, msg):

    print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")


def on_subscribe(mosq, obj, mid, granted_qos):

    print("Subscribed OK")


def on_unsubscribe(mosq, obj, mid, granted_qos):

    print("Unsubscribed OK")

# Set callbacks

mqttc.on_message = on_message

mqttc.on_connect = on_connect

mqttc.on_subscribe = on_subscribe

mqttc.on_unsubscribe = on_unsubscribe


# Connect and subscribe to MQTT

print("Connecting to " + host )

mqttc.connect(host, port=1883, keepalive=60)

#xbee settings

s.write("+++".encode())

char = s.read(2)

print("Enter AT mode.")

print(char.decode())


s.write("ATMY 0x140\r\n".encode())

char = s.read(3)

print("Set MY 0x140.")

print(char.decode())


s.write("ATDL 0x240\r\n".encode())

char = s.read(3)

print("Set DL 0x240.")

print(char.decode())


s.write("ATID 0x1\r\n".encode())

char = s.read(3)

print("Set PAN ID 0x1.")

print(char.decode())


s.write("ATWR\r\n".encode())

char = s.read(3)

print("Write config.")

print(char.decode())


s.write("ATMY\r\n".encode())

char = s.read(4)

print("MY :")

print(char.decode())


s.write("ATDL\r\n".encode())

char = s.read(4)

print("DL : ")

print(char.decode())


s.write("ATCN\r\n".encode())

char = s.read(3)

print("Exit AT mode.")

print(char.decode())


print("start sending RPC")


#start write

s.write(bytes("\r", 'UTF-8'))

line=s.readline() # Read an echo string from K66F terminated with '\n' (pc.putc())

print(line)

line=s.readline() # Read an echo string from K66F terminated with '\n' (RPC reply)

print(line)

t.sleep(1)

action=[]

# Store each action send by K66F
while(line!="end"):
    #s.write(bytes("/getData/run\n", 'UTF-8'))
    print("read line")
    line=s.readline() # Read an echo string from K66F terminated with '\n' (pc.putc)
    print(line)
    # line=s.readline() # Read an echo string from K66F terminated with '\n' (RPC reply)
    # print(line)
    if(line != '\0' or "end") action.append(line)
    print("action="+str(line))
    t.sleep(1)


print("end")
#publish log from xbee to mqtt
for i in range(0,count):
    print("mqtt publish")
    mqttc.publish("action", action[i]) #mqttc.publish(topic,msg)
    print("send action"+str(action[i]))

    #t.sleep(0.1)

# plt.subplot(111)
# plt.plot(num_time, dataNum, color='blue',linestyle='-')
# plt.xlabel('TimeStamp')
# plt.ylabel('Collected Data Times')
# plt.show()


    
    
