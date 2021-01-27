#!/usr/bin/env python
import sys
import shutil
import paramiko
import subprocess
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from ui_to_py import Ui_sp

import rospy
from geometry_msgs.msg import Twist

class Actt_gui(QDialog):
   def __init__(self,parent=None):

      super(Actt_gui, self).__init__(parent)
      self.ui = Ui_sp()
      self.ui.setupUi(self)

   def ros_init(self):
      a=1

   def ssh_client(self):
      self.proc_mapping = subprocess.Popen(["/bin/sh", "actt_ntp.sh"])
      self.client_actt = paramiko.SSHClient()
      self.client_actt.set_missing_host_key_policy(paramiko.AutoAddPolicy())
      self.client_actt.connect('RasPiのIPアドレス', username='pi', password='RasPiのpassword')

      self.stdin, self.stdout, self.stderr = self.client_actt.exec_command('source actt.sh')
#      self.stdin, self.stdout, self.stderr = self.client_actt.exec_command('ls -al')
      #print(self.stdin)
      #for line in self.stderr:
      #   print(line)
      #print(self.stderr)

   def ssh_dis(self):
      subprocess.Popen(["rosnode", "kill", "/rplidarNode"])
      subprocess.Popen(["rosnode", "kill", "/actt_joy"])
      self.client_actt.close()

   def mapping(self):
      self.proc_mapping = subprocess.Popen(["/bin/sh", "actt_mapping.sh"])
      #self.proc_mapping = subprocess.Popen(['ls'], shell=True)

   def save_map(self):
      self.proc_save = subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", "mymap"])

   def load_map(self):
      shutil.copyfile("mymap.yaml", "catkin_actt_pc/src/actt_pc/maps/mymap.yaml")
      shutil.copyfile("mymap.pgm",  "catkin_actt_pc/src/actt_pc/maps/mymap.pgm")

      #self.proc_server = subprocess.Popen(["roslaunch", "actt_pc", "actt_map_server.launch"])

   def fin_mapping(self):
      subprocess.Popen(["rosnode", "kill", "/rplidarNode"])
      subprocess.Popen(["rosnode", "kill", "/actt_joy"])
      subprocess.Popen(["rosnode", "kill", "/actt_pc"])
      subprocess.Popen(["rosnode", "kill", "/map_2_odom"])
      subprocess.Popen(["rosnode", "kill", "/base_footprint_2_base_link"])
      subprocess.Popen(["rosnode", "kill", "/base_link_2_laser_link"])
      subprocess.Popen(["rosnode", "kill", "/laser_scan_matcher_node"])
      subprocess.Popen(["rosnode", "kill", "/joy_node"])
      subprocess.Popen(["rosnode", "kill", "/hector_geotiff_node"])
      subprocess.Popen(["rosnode", "kill", "/hector_mapping"])
      subprocess.Popen(["rosnode", "kill", "/hector_trajectory_server"])
      subprocess.Popen(["rosnode", "kill", "/robot_state_publisher"])
      subprocess.Popen(["rosnode", "kill", "/rviz"])

   def navigation(self):
      #self.proc_navigation = subprocess.Popen(["roslaunch", "actt_pc", "actt_navigation.launch"])
      self.proc_mapping = subprocess.Popen(["/bin/sh", "actt_navigation.sh"])

   def stop(self):
      #self.proc_save.kill()
      subprocess.Popen(["rosnode", "kill", "/rplidarNode"])
      subprocess.Popen(["rosnode", "kill", "/actt_joy"])
      subprocess.Popen(["rosnode", "kill", "/actt_pc"])
      subprocess.Popen(["rosnode", "kill", "/amcl"])
      subprocess.Popen(["rosnode", "kill", "/base_footprint_2_base_link"])
      subprocess.Popen(["rosnode", "kill", "/base_link_2_laser_link"])
      subprocess.Popen(["rosnode", "kill", "/laser_scan_matcher_node"])
      subprocess.Popen(["rosnode", "kill", "/map_server"])
      subprocess.Popen(["rosnode", "kill", "/move_base"])
      subprocess.Popen(["rosnode", "kill", "/robot_state_publisher"])
      subprocess.Popen(["rosnode", "kill", "/rosout"])
      subprocess.Popen(["rosnode", "kill", "/rviz"])
      subprocess.Popen(["rosnode", "kill", "/actt_type_changer"])
      subprocess.Popen(["rosnode", "kill", "/actt_white_line_detection"])
      subprocess.Popen(["rosnode", "kill", "/frame_pub"])
      subprocess.Popen(["rosnode", "kill", "/image_republish"])

if __name__ == '__main__':
   rospy.init_node('run')
   app = QApplication(sys.argv)
   window = Actt_gui()
   window.show()
   sys.exit(app.exec_())

