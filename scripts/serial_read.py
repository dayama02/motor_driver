#!/usr/bin/env python
import rospy
import serial
from math import sin, cos, pi, atan2, hypot
from geometry_msgs.msg import Twist


class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readLine(self):
        i = self.buf.find(b"\\n")
        if i>= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i>= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

class MotorDriver:
    def __init__(self):
        self.wheel_track = 1
        self.ser = serial.Serial("/dev/ttyUSB1", 115200, timeout=1)
        self.sub = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        self.now = rospy.Time.now()
        self.last = rospy.Time.now()
        self.last_cmd_vel = rospy.Time.now()
        self.read_line = ReadLine(self.ser)
        print("motor_driver initialized")

    def cmdVelCallback(self, cmd):
        print ("hoge")

    def run(self):
        self.now = rospy.Time.now()
        if self.ser.is_open:
            # todo: read current velocity from arduino
            message = self.read_line.readLine()
            print(message)

            # monitor cmd_vel timeouts
            # if(self.last_cmd_vel.to_sec() > 2):
            #     self.last_cmd_vel = rospy.Time.now()
            #     speed_cmd_left  = "{:1.1f}".format(0)
            #     speed_cmd_right = "{:1.1f}".format(0)

            #     self.ser.write(speed_cmd_left + "," + speed_cmd_right + "\n")
        
        else:
            print("serial device is closed")
        
        # calclate delta t
        dt = self.now - self.last
        dt = dt.to_sec()
        self.last = self.now
        

def main():
    rospy.init_node("arduino_motor_driver")
    rate = rospy.Rate(10)
    driver = MotorDriver()

    if not driver.ser:
        print("[arduino_motor_driver]: cannot open serial device")
        exit()

    while not rospy.is_shutdown():
        driver.run()
        rate.sleep()
    
    print("exiting arduino_motor_driver")
    if driver.ser:
        if driver.ser.is_open:
            driver.ser.close()


if __name__ == '__main__':
  main()