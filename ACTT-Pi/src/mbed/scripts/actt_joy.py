#!/usr/bin/env python
# coding: UTF-8

import rospy
import numpy as np
import serial
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

SET_POW = 0.51

actt = serial.Serial('/dev/ttyAMA0', 9600)

Status = 'RUN'
Type   = 0
cnt    = 0

pow_joy = 0
servo_joy = 0

pow_vel = 0
servo_cmd = 0
gain = 0
servo_vel_z = 0

def msg_callback(msg):
   global Type, cnt

   if msg.data == 'AWLD0':
      cnt = cnt + 1
      Type = 1

   elif msg.data == 'AUTO0':
      cnt = cnt + 1
      Type = 2

   if cnt == 2:
      Type = 3

def joy_callback(pos):
   global pow_joy, servo_joy

   pow_joy   = pos.axes[1]
   servo_joy = pos.axes[2]

   run()

def vel_callback(vel):
   global pow_vel, servo_vel_x, servo_vel_z

   pow_vel     = vel.linear.x
   servo_vel_z = vel.angular.z

   #run()

def servo_callback(servo):
   global servo_cmd, gain

   servo_cmd = servo.x
   gain      = servo.z

   #run()

def scan_callback(scan):
   global Status

   min_ranges = 1.0

   for i in range(180-20, 180+20):
      if min_ranges > scan.ranges[i]:
         min_ranges = scan.ranges[i]

   # 20cm以内に障害物が正面にあった場合, 緊急停止. PCとACTTに通信不良が生じた場合の対策.
   if min_ranges < 0.20:
      Status = 'Emergency'

   run()

def run():
   global pow_joy, servo_joy, pow_vel, servo_cmd, gain, servo_vel_z, Status, Type

   E_flag = 0
   buzz   = 201

   if Status == 'RUN':
      # コントローラ入力によるマイコンへの出力
      if Type == 0:
         pow   = int(pow_joy * (-120)) + 120
         servo = int(servo_joy * 90) + 90

      # コントローラ入力と白線検知によるマイコンへの出力
      elif Type == 1:
         pow   = int(pow_joy * (-120)) + 120
         if gain == 0:
            servo = 90 + int((90 - int(servo_cmd)) * 1.45)  # 0 -- 90 -- 180
         elif gain == 1:
            servo = 90 + int((90 - int(servo_cmd)) * 1.6)  # 0 -- 90 -- 180

      # 速度コマンド(move_base)によるマイコンへの出力
      elif Type == 2:
         if pow_vel >= 0.1:
            #pow = int((pow_vel + 0.3) * (-120)) + 120
            pow = int(SET_POW * (-120)) + 120
         elif pow_vel <= -0.1:
            #pow = int((pow_vel - 0.) * (-120)) + 120
            pow = int(-SET_POW * (-120)) + 120
         else:
            pow = 120
         servo = int((servo_vel_z*2.5) * 90) + 90   # -1 --- 1

      # 速度コマンド(前後進)と白線検知によるマイコンへの出力
      elif Type == 3:
         if pow_vel >= 0.1:
            pow = int(SET_POW * (-120)) + 120
         elif pow_vel <= -0.1:
            pow = int(-SET_POW * (-120)) + 120
         else:
            pow = 120
         if gain == 0:
            servo = 90 + int((90 - int(servo_cmd)) * 1.45)  # 0 -- 90 -- 180
         elif gain == 1:
            servo = 90 + int((90 - int(servo_cmd)) * 1.6)  # 0 -- 90 -- 180

   elif Status == 'Emergency':
      pow    = 120
      servo  = 90
      Status = 'RUN'
      buzz   = 200
      E_flag = 1

   rospy.loginfo("servo = %0.3f", servo)

   actt.write(chr(128))
   actt.write(chr(pow))
   actt.write(chr(servo))

   if E_flag == 1:
      rospy.sleep(5.0)


def listener():
   rospy.init_node('actt_joy', anonymous=True)
   rospy.Subscriber('/chatter', String, msg_callback)
   rospy.Subscriber('/joy', Joy, joy_callback)
   rospy.Subscriber('/cmd_vel', Twist, vel_callback)
   rospy.Subscriber('/cmd_servo', Vector3, servo_callback)
   rospy.Subscriber('/scan', LaserScan, scan_callback)

   rospy.spin()

if __name__ == '__main__':
   listener()
