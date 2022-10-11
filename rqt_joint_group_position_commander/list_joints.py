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
from typing import List

def list_joints(controllers: List[ControllerState]) -> List[str]:
    """List the joints claimed by any controller for the given controller_manager."""
    joints: List[str] = []
    for controller in controllers:
        if controller.type == 'position_controllers/JointGroupPositionController':
            for interface in controller.claimed_interfaces:
                # Get joint name from interface (eg. 'joint_foo/position' -> 'joint_foo')
                joints.append(interface.split('/')[0])
    return joints
