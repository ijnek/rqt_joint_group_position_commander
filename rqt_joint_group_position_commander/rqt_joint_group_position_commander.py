import os

from ament_index_python.packages import get_package_share_directory

from controller_manager_msgs.srv import ListControllers
from controller_manager.controller_manager_services import list_controllers
from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSignal, pyqtSlot, Qt, QTimer, Slot
from python_qt_binding.QtWidgets import QLabel, QSlider, QWidget
from qt_gui.plugin import Plugin
from qt_gui.plugin_context import PluginContext
from rclpy import spin_until_future_complete
from rclpy.logging import get_logger
from rclpy.qos import DurabilityPolicy, QoSProfile
from rqt_joint_group_position_commander.list_joints import list_joints
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from typing import List, Tuple
from urdf_parser_py import urdf
import time

class RqtJointGroupPositionCommander(Plugin):

    # A value to divide the slider value by to get the joint position.
    # This is necessary because slider values only allow integers and not floats.
    SLIDER_MULTIPLIER = 1000

    _joints_claimed_by_controller: List[str] = []
    _joints: List[urdf.Joint] = []
    _sliders: List[Tuple[QLabel, QSlider]] = []
    _context: PluginContext
    _widget: QWidget

    update_joints = pyqtSignal()

    _log = get_logger('rqt_joint_group_position_commander')

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("RqtJointGroupPositionCommander")

        self._context = context

        # Add widget
        self._add_widget(context)

        # Connect signal
        self.update_joints.connect(self._update_sliders)

        # Create publisher
        self._cmd_pub = context.node.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 1)

        # Create subscription
        self._robot_description_sub = context.node.create_subscription(
            String, '/robot_description', self._callback_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._joint_state_sub = context.node.create_subscription(
            JointState, '/joint_states', self._callback_joint_states, 1)

        # Configs (Get these from configuration dialog)
        controller_manager_name = 'controller_manager'

        self._log.debug('Constructed RqtJointGroupPositionCommander')

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def trigger_configuration(self):
        self._log.debug('Opening configuration dialog')
        client = self._context.node.create_client(ListControllers, '/controller_manager/list_controllers')
        while not client.wait_for_service(timeout_sec=1.0):
            self._log.debug('List Controller service not available, waiting again...')
        self.future = client.call_async(ListControllers.Request())
        spin_until_future_complete(self._context.node, self.future)
        controllers = self.future.result()
        client.destroy()
        self._log.debug('List joints claimed by controller')
        self._joints_claimed_by_controller = list_joints(controllers.controller)
        self._log.debug('Found {} joints claimed by controllers: {}'.format(
            len(self._joints_claimed_by_controller), self._joints_claimed_by_controller))
        self._update_sliders()

    def _add_widget(self, context):
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

    def publish(self):
        msg = Float64MultiArray()

        for slider in self._sliders:
            msg.data.append(float(slider[1].value()) / self.SLIDER_MULTIPLIER)

        # for slider in self._sliders:
        #     if slider[0].text() == 'base_link__arm1':
        #         msg.data.append(float(slider[1].value()) / self.SLIDER_MULTIPLIER)

        # for slider in self._sliders:
        #     if slider[0].text() == 'arm1__arm2':
        #         msg.data.append(float(slider[1].value()) / self.SLIDER_MULTIPLIER)

        # for slider in self._sliders:
        #     if slider[0].text() == 'arm2__brush':
        #         msg.data.append(float(slider[1].value()) / self.SLIDER_MULTIPLIER)

        self._cmd_pub.publish(msg)

    def _callback_robot_description(self, msg: String):
        self._log.debug('Received robot description')
        robot = urdf.URDF.from_xml_string(msg.data)

        self._joints = robot.joints
        self._log.debug('Found {0} joints from URDF'.format(len(self._joints)))

        self.update_joints.emit()

    def _callback_joint_states(self, msg: JointState):
        self._log.debug('Received joint states')

        # If sliders aren't initialized, don't update them
        if not self._sliders:
            return

        self._log.debug('Applying joint states to slider positions')
        for i, joint in enumerate(msg.name):
            for slider in self._sliders:
                if slider[0].text() == joint:
                    self._log.debug('Updating slider for joint {}'.format(joint))
                    slider[1].blockSignals(True)
                    slider[1].setValue(msg.position[i] * self.SLIDER_MULTIPLIER)
                    slider[1].blockSignals(False)

        self._log.debug('Destroying joint states subscriber')
        self._joint_state_sub.destroy()

    @pyqtSlot()
    def _update_sliders(self):
        self._log.debug('Updating sliders')

        # Clear out layout and empty list of sliders
        self._context.remove_widget(self._widget)
        self._add_widget(self._context)

        # print(self._joints)
        for joint in self._joints:
            if joint.type == 'revolute':
                if joint.name in self._joints_claimed_by_controller:
                    self._add_slider(joint.name, joint.limit.lower, joint.limit.upper)

        self._log.debug('Finished updating sliders')

    def _add_slider(self, name: str, minimum: float, maximum: float):
        label = QLabel(name)

        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setMinimum(minimum * self.SLIDER_MULTIPLIER)
        slider.setMaximum(maximum * self.SLIDER_MULTIPLIER)
        slider.valueChanged.connect(self.publish)

        self._widget.layout().addRow(label, slider)
        self._sliders.append((label, slider))
