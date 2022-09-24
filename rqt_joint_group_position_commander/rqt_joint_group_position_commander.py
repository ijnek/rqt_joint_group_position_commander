import os

from ament_index_python.packages import get_package_share_directory

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QSlider, QWidget
from std_msgs.msg import Float64MultiArray

class RqtJointGroupPositionCommander(Plugin):

    _joint_states = [0.0, 0.0, 0.0]

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

        # Connect callbacks
        self._widget.slider_1.valueChanged.connect(self.callback_joint_1)
        self._widget.slider_2.valueChanged.connect(self.callback_joint_2)
        self._widget.slider_3.valueChanged.connect(self.callback_joint_3)

        # Create publisher
        self._cmd_pub = context.node.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 1)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def callback_joint_1(self, val):
        self._joint_states[0] = val / 1000.0
        self.publish()

    def callback_joint_2(self, val):
        self._joint_states[1] = val / 1000.0
        self.publish()

    def callback_joint_3(self, val):
        self._joint_states[2] = val / 1000.0
        self.publish()

    def publish(self):
        msg = Float64MultiArray()
        msg.data = self._joint_states
        self._cmd_pub.publish(msg)
