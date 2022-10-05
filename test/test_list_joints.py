# Copyright 2022 Kenji Brameld
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from controller_manager_msgs.msg import ControllerState
from rqt_joint_group_position_commander.list_joints import list_joints
from typing import List

def test_list_joints():
    controllers: List[ControllerState] = [
        ControllerState(
            type='position_controllers/JointGroupPositionController',
            claimed_interfaces=['joint_a/position', 'joint_b/position'],
        ),
        ControllerState(
            type='position_controllers/JointGroupPositionController',
            claimed_interfaces=['joint_c/position'],
        ),
        ControllerState(
            type='other_type',
            claimed_interfaces=['joint_d/position'],
        ),
    ]
    joints = list_joints(controllers)
    assert joints == ['joint_a', 'joint_b', 'joint_c']
