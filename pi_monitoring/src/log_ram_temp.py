#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Log RAM and Temp of RPI and publish as ROS topics.
#tony.jacob@uri.edu

import os
import rospy
import time
from std_msgs.msg import Float64MultiArray

class Log_RAM_Temp:
    def __init__(self) -> None:
        self.pub_rpi_info = rospy.Publisher("/alpha_rise/pi/monitor", Float64MultiArray, queue_size=1)

    def get_cpu_temp(self):
        temp = os.popen("vcgencmd measure_temp").readline()
        return temp.replace("temp=", "").strip()
    
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
    
    def collect_and_publish(self):
        array = Float64MultiArray()
        temp = self.get_cpu_temp()
        ram_total, ram_used = self.get_ram_usage()
        array.data = [temp,ram_total,ram_used]

        self.pub_rpi_info.publish(array)
        time.sleep(60)
    
if __name__ == "__main__":
    rospy.init_node("pi_monitor_node")
    Log_RAM_Temp()
    rospy.spin()