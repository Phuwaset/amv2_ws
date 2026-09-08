#!/usr/bin/env python3
import robot_upstart
#system_start = robot_upstart.Job(name="amv-start", interface="wlx00c0caae7d00", master_uri="http://amv2-wifi:11311")
system_start = robot_upstart.Job(name="amv-start")
system_start.symlink = False
system_start.add(package="amv_navigation", filename="launch/amv_navigation.launch")
system_start.install()
