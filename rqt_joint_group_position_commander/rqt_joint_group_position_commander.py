import os

from ament_index_python.packages import get_package_share_directory

from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot, Qt
from python_qt_binding.QtWidgets import QLabel, QSlider, QWidget
from qt_gui.plugin import Plugin
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String
from typing import List, Tuple
from urdf_parser_py.urdf import URDF

class RqtJointGroupPositionCommander(Plugin):

    # A value to divide the slider value by to get the joint position.
    # This is necessary because slider values only allow integers and not floats.
    SLIDER_MULTIPLIER = 1000

    _joints: URDF = None
    _sliders: List[Tuple[QLabel, QSlider]] = []

    update_joints = pyqtSignal()

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("RqtJointGroupPositionCommander")

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        ui_file = os.path.join(
            get_package_share_directory("rqt_joint_group_position_commander"),
            "resource",
            "rqt_joint_group_position_commander.ui",
        )
        loadUi(ui_file, self._widget)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (" (%d)" % context.serial_number())
            )
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Connect signal
        self.update_joints.connect(self._update_sliders)

        # Create publisher
        self._cmd_pub = context.node.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 1)

        # Create subscription
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos_profile.reliability = ReliabilityPolicy.RELIABLE
        self._robot_description_sub = context.node.create_subscription(
            String, '/robot_description', self._callback_robot_description, qos_profile)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def publish(self):
        msg = Float64MultiArray()
        for slider in self._sliders:
            msg.data.append(float(slider.value()) / self.SLIDER_MULTIPLIER)
        self._cmd_pub.publish(msg)

    def _callback_robot_description(self, msg: String):
        robot = URDF.from_xml_string(msg.data)
        self._joints = robot.joints
        self.update_joints.emit()

    @pyqtSlot()
    def _update_sliders(self):
        self._sliders.clear()
        for joint in self._joints:
            if joint.type == 'revolute':
                self._add_slider(joint.name, joint.limit.lower, joint.limit.upper)

    def _add_slider(self, name: str, minimum: float, maximum: float):
        label = QLabel(name)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setMinimum(minimum * self.SLIDER_MULTIPLIER)
        slider.setMaximum(maximum * self.SLIDER_MULTIPLIER)
        slider.valueChanged.connect(self.publish)

        self._widget.layout().addRow(label, slider)
        self._sliders.append(slider)
