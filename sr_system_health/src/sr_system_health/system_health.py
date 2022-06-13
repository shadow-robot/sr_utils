#!/usr/bin/env python3

# Copyright 2022 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import psutil

import rospy

from sr_system_health.msg import SystemHealth


class SystemHealthCheck:

    def __init__(self):

        self._sampling_period = rospy.get_param("~sampling_period", 1)

        self._timer = rospy.Timer(rospy.Duration(self._sampling_period), self._timer_cb)
        self._publisher = rospy.Publisher("/system_health", SystemHealth, queue_size=10)

    def _timer_cb(self, event):

        msg = SystemHealth()

        msg.stamp = rospy.Time().now()
        msg.cpu_usage = psutil.cpu_percent()
        msg.percpu_usage = psutil.cpu_percent(percpu=True)

        msg.cpu_frequency = psutil.cpu_freq().current

        cpu_frequencies = psutil.cpu_freq(percpu=True)
        cpu_freq_list = []
        for cpu_freq in cpu_frequencies:
            cpu_freq_list.append(cpu_freq.current)
        msg.percpu_frequency = cpu_freq_list
        
        core_temps = psutil.sensors_temperatures()["coretemp"]
        core_temp_list = []
        for core_temp in core_temps:
            core_temp_list.append(core_temp.current)
        msg.percore_temp = core_temp_list

        msg.ram_usage = psutil.virtual_memory().percent
        msg.disk_usage = psutil.disk_usage('/').percent

        self._publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node('system_health_node')
    system_health_check = SystemHealthCheck()
    rospy.spin()
