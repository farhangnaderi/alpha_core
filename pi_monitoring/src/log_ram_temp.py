#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Log RAM and Temp of RPI and publish as ROS topics.
#sudo apt install libraspberrypi-bin
#sudo usermod -aG video <username>
#tony.jacob@uri.edu

import os
import rospy
import psutil
from std_msgs.msg import Float64
from sensor_msgs.msg import Temperature

class Log_RAM_Temp:
    def __init__(self) -> None:
        self.pub_ram_info = rospy.Publisher("/alpha_rise/pi/ram_utilized", Float64, queue_size=1)
        self.pub_cpu_temp_info = rospy.Publisher("/alpha_rise/pi/cpu/temp", Temperature, queue_size=1)
        self.pub_cpu_usage_info = rospy.Publisher("/alpha_rise/pi/cpu/utilized", Float64, queue_size=1)

        rospy.loginfo("Pi Monitoring started")
        self.rate = rospy.Rate(1)
        self.collect_and_publish()

    def get_cpu_temp(self):
        temp = os.popen("vcgencmd measure_temp").readline()
        return float(temp.replace("temp=", "").replace("'C", "").strip())
    
    def get_ram_usage(self):    
        meminfo = {}
        with open('/proc/meminfo') as f:
            for line in f:
                parts = line.split(':')
                meminfo[parts[0]] = int(parts[1].strip().split()[0])
        mem_total = meminfo['MemTotal']
        mem_available = meminfo['MemAvailable']
        mem_used = mem_total - mem_available
        return mem_total, mem_used
    
    def get_cpu_usage(self):
        return psutil.cpu_percent()
    
    def collect_and_publish(self):
        temp_msg = Temperature()
        while not rospy.is_shutdown():
            #Temperature
            temp = self.get_cpu_temp()
            temp_msg.header.stamp = rospy.Time()
            temp_msg.temperature = temp
            #RAM
            ram_total, ram_used = self.get_ram_usage()
            #CPU
            cpu_used = self.get_cpu_usage()

            self.pub_ram_info.publish((ram_used/ram_total) *100)
            self.pub_cpu_temp_info.publish(temp)
            self.pub_cpu_usage_info.publish(cpu_used)
            self.rate.sleep()
    
if __name__ == "__main__":
    rospy.init_node("pi_monitor_node")
    Log_RAM_Temp()
    rospy.spin()