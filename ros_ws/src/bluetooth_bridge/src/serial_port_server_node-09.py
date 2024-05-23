#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
 
import rospy
import std_msgs.msg
import serial
import time 
import threading 
from std_msgs.msg import Int32
from std_msgs.msg import String


# topics
topic_from_bluetooth="/bluetooth/received/data"
#topic_from_auto_driver="/auto_driver/received/data"
topic_from_auto_driver_direction="/auto_driver/send/direction"
topic_from_auto_driver_speed="/auto_driver/send/speed"
topic_from_auto_driver_gear="/auto_driver/send/gear"
topic_from_auto_driver_beef="/auto_driver/send/beef"

topic_from_vcu="/bluetooth/send"
#topic_direction = "/bluetooth/received/direction"        # Received data from Bluetooth will
#topic_speed = "/bluetooth/received/speed"        # be published to this topic.
#topic_gear = "/bluetooth/received/gear"
#topic_manual = "/bluetooth/received/manual"
#topic_beep = "/bluetooth/received/beep"

#serial init
node_name='serial_port' 
serialPort = "/dev/ttyUSB0"
baudRate = 1000000
#ser = serial.Serial(serialPort, baudRate, timeout=0.5)
ser = serial.Serial(serialPort, baudRate)
print("serial port is %s ,baudRate is %d" % (serialPort, baudRate))
time.sleep(1)
#default          head,direction,speed,gear,manul,beef,crc,bit
#auto_driver_data=[0xaa,50       ,0    ,3   ,0    ,0   ,0  , 0]
auto_driver_data=chr(0xaa)+chr(50)+chr(0)+chr(3)+chr(0)+chr(0)+chr(0)+chr(0)

flag_manul=0
 
#callback relate to subscriber 
def callback_bluetooth(message):
    global flag_manul
    print("bluetooth_data:",message.data)
    flag_manul=ord(message.data[4])
    if flag_manul==1:
        print "Received from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(message.data[0]),ord(message.data[1]),ord(message.data[2]),ord(message.data[3]),ord(message.data[4]),ord(message.data[5]),ord(message.data[6]))
        ser.write(message.data[0:8])
        ser.flush()
def callback_direction(message):
    global flag_manul
    global auto_driver_data
    print("auto_driver_Direction:",message.data,chr(message.data))
   # auto_driver_data[1]=chr(message.data)
    auto_driver_data =auto_driver_data[0]+chr(message.data)+auto_driver_data[2:] 
    crc=chr(ord(auto_driver_data[0])^ord(auto_driver_data[1])^ord(auto_driver_data[2])^ord(auto_driver_data[3])^ord(auto_driver_data[4])^ord(auto_driver_data[5]))
    auto_driver_data = auto_driver_data[0:6]+crc+auto_driver_data[7:]
    if flag_manul==0:
        print("Received from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(auto_driver_data[0]),ord(auto_driver_data[1]),ord(auto_driver_data[2]),ord(auto_driver_data[3]),ord(auto_driver_data[4]),ord(auto_driver_data[5]),ord(auto_driver_data[6])))
        ser.write(auto_driver_data[0:8])
        ser.flush()

def callback_speed(message):
    global flag_manul
    global auto_driver_data
    print("auto_driver_speed:",message)
#    auto_driver_data[2]=str(message.data)
    auto_driver_data =auto_driver_data[0:2]+chr(message.data)+auto_driver_data[3:] 
    crc=chr(ord(auto_driver_data[0])^ord(auto_driver_data[1])^ord(auto_driver_data[2])^ord(auto_driver_data[3])^ord(auto_driver_data[4])^ord(auto_driver_data[5]))
    auto_driver_data = auto_driver_data[0:6]+crc+auto_driver_data[7:]
    if flag_manul==0:
        print("Receiived from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(auto_driver_data[0]),ord(auto_driver_data[1]),ord(auto_driver_data[2]),ord(auto_driver_data[3]),ord(auto_driver_data[4]),ord(auto_driver_data[5]),ord(auto_driver_data[6])))
        ser.write(auto_driver_data[0:8])
        ser.flush()
def callback_gear(message):
    global flag_manul
    global auto_driver_data
    print("auto_driver_gear:",message.data)
 #   auto_driver_data[3]=str(message.data)
    auto_driver_data =auto_driver_data[0:3]+chr(message.data)+auto_driver_data[4:] 
    crc=chr(ord(auto_driver_data[0])^ord(auto_driver_data[1])^ord(auto_driver_data[2])^ord(auto_driver_data[3])^ord(auto_driver_data[4])^ord(auto_driver_data[5]))
    auto_driver_data = auto_driver_data[0:6]+crc+auto_driver_data[7:]
    if flag_manul==0:
        print("Received from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(auto_driver_data[0]),ord(auto_driver_data[1]),ord(auto_driver_data[2]),ord(auto_driver_data[3]),ord(auto_driver_data[4]),ord(auto_driver_data[5]),ord(auto_driver_data[6])))
        ser.write(auto_driver_data[0:8])
        ser.flush()
def callback_beep(message):
    global flag_manul
    global auto_driver_data
    print("auto_driver_beep:",message.data)
  #  auto_driver_data[5]=str(message.data)
    auto_driver_data =auto_driver_data[0:5]+chr(message.data)+auto_driver_data[6:] 
    crc=chr(ord(auto_driver_data[0])^ord(auto_driver_data[1])^ord(auto_driver_data[2])^ord(auto_driver_data[3])^ord(auto_driver_data[4])^ord(auto_driver_data[5]))
    auto_driver_data = auto_driver_data[0:6]+crc+auto_driver_data[7:]
    if flag_manul==0:
        print("Received from bluetooth: header=%d,direction=%d,speed=%d,gear==%d,manual=%d,beep=%d,crc=%d" %(ord(auto_driver_data[0]),ord(auto_driver_data[1]),ord(auto_driver_data[2]),ord(auto_driver_data[3]),ord(auto_driver_data[4]),ord(auto_driver_data[5]),ord(auto_driver_data[6])))
        ser.write(auto_driver_data[0:8])
        ser.flush()

def thread_job():
    rospy.spin()
 
 
def listener():
    #node init
    rospy.init_node(node_name, anonymous=True)  
    print("init_node:",node_name)
    #publish topic
    vcu_data_pub = rospy.Publisher(topic_from_vcu, String, queue_size = 10)
    #subscriber topic
    rospy.Subscriber(topic_from_bluetooth, String, callback_bluetooth)
    #rospy.Subscriber(topic_from_auto_driver, String, callback_auto_driver)
    rospy.Subscriber(topic_from_auto_driver_direction, Int32, callback_direction) 
    rospy.Subscriber(topic_from_auto_driver_speed, Int32, callback_speed) 
    rospy.Subscriber(topic_from_auto_driver_gear, Int32, callback_gear) 
    #rospy.Subscriber(topic_manual, Int32, callback_manual) 
    rospy.Subscriber(topic_from_auto_driver_beef, Int32, callback_beep) 
    #
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        get_str = ser.readline()
        print(get_str)
        get_str =get_str.strip()
        print(get_str)
#        vcu_data_pub.publish(get_str)
        print("serial_port publish",get_str)
        rate.sleep()
        
 
 
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("enter except,Subscriber")
 
 
 
########################
 
