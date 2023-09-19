#!/usr/bin/env python
# -*- coding: utf-8 -*-

import robotchain as framework

def robot_start(sdk):
    sdk.utils.log.debug(sdk.ros.client.is_connected)
    sdk.ros.subscribe("/serial_output")
    while True:
        send_message = {"type": "led-status", "channel": "all", "status": "switch"}
        sdk.ros.publisher("/serial_input", send_message)
        sdk.utils.time.sleep(1)
        send_message = {"type": "key-status"}
        sdk.ros.publisher("/serial_input", send_message)
        sdk.utils.time.sleep(1)

def robot_message(sdk, message):
    sdk.utils.log.debug(message)

def robot_exit(sdk):
    send_message = {"type": "led-status", "channel": "all", "status": "off"}
    sdk.ros.publisher("/serial_input", send_message)

if __name__ == '__main__':
    framework_sdk = framework.Init()
