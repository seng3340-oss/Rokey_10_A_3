from __future__ import annotations

from typing import Optional

import threading
import json
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


from dsr_msgs2.srv import (
    MoveHome,
    MoveJoint,
    MoveLine,
    MoveSplineTask,
    MoveCircle,
    MoveJointx,
    GetCurrentPosx,
    GetCurrentPosj,
    GetSolutionSpace,
)
from rcl_interfaces.srv import GetParameters
from onrobot_rg_msgs.srv import SetCommand

from .drawing_motion import (
    DEFAULT_ACC_ANGULAR,
    DEFAULT_ACC_LINEAR,
    DEFAULT_VEL_ANGULAR,
    DsrPos,
    MotionType,
)

from doosan_robot_client import DoosanRobotClient


client = DoosanRobotClient()
client.gripper_open()