#!/usr/bin/env python3

# Copyright 2021 Shadow Robot Company Ltd.
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

    def __init__(self, frequency=1):

        self._frequency = frequency
        self._timer = rospy.Timer(rospy.Duration(self._frequency), self._timer_cb)

        self._publisher = rospy.Publisher("/system_health", SystemHealth, queue_size=10)

    def _timer_cb(self, event):
        
        msg = SystemHealth()

        msg.stamp = rospy.Time().now()
        msg.cpu_usage = psutil.cpu_percent()
        msg.cpu_frequency  = psutil.cpu_freq().current
        msg.percpu_usage = psutil.cpu_percent(percpu=True)
        msg.ram_usage = psutil.virtual_memory().percent
        msg.disk_usage = psutil.disk_usage('/').percent

        self._publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node("system_health_check")
    system_health_check = SystemHealthCheck()
    rospy.spin()
