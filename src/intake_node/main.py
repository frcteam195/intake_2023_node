#!/usr/bin/env python3

from threading import Thread
import rospy

from ck_ros_msgs_node.msg import Intake_Control, Intake_Status
from intake_node.intake_simulation import IntakeSimulation

from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *

from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode, BufferedROSMsgHandlerPy


class IntakeNode():
    """
    The Intake Node.
    """

    def __init__(self) -> None:
        self.intake_simulation = IntakeSimulation()

        self.control_subscriber = BufferedROSMsgHandlerPy(Intake_Control)
        self.control_subscriber.register_for_updates("IntakeControl")

        self.status_publisher = rospy.Publisher(name="IntakeStatus", data_class=Intake_Status, queue_size=50, tcp_nodelay=True)

        self.intakeRollerMotor = Motor("intake", MotorType.TalonFX)
        self.pincherSolenoid = Solenoid("pincher", SolenoidType.SINGLE)

        register_for_robot_updates()
        t1 = Thread(target=self.loop)
        t1.start()

        rospy.spin()

        t1.join(5)

    def loop(self) -> None:
        """
        Periodic function for the Intake Node.
        """

        rate = rospy.Rate(50)
        frame_count = 0

        while not rospy.is_shutdown():

            if robot_status.get_mode() == RobotMode.DISABLED:
                    self.intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                    self.pincherSolenoid.set(SolenoidState.OFF)
            else:
                if self.control_subscriber.get() is not None:
                    intake_ctrl_msg: Intake_Control = self.control_subscriber.get()

                    if intake_ctrl_msg.rollers_intake:
                        self.intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 1.0, 0.0)
                    elif intake_ctrl_msg.rollers_outtake:
                        self.intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, -0.15, 0.0)
                    else:
                        self.intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)

                    if intake_ctrl_msg.pinched:
                        self.pincherSolenoid.set(SolenoidState.OFF)
                    else:
                        self.pincherSolenoid.set(SolenoidState.ON)

            if frame_count % 10 is 0:
                self.intake_simulation.publish_intake_1_link(self.pincherSolenoid.get() == SolenoidState.OFF)
                self.intake_simulation.publish_intake_2_link(self.pincherSolenoid.get() == SolenoidState.OFF)
                self.intake_simulation.publish_arrow_link(90, self.intakeRollerMotor.get_sensor_velocity())

            status_message = Intake_Status()
            status_message.pinched = self.pincherSolenoid.get() == SolenoidState.OFF
            self.status_publisher.publish(status_message)

            frame_count += 1
            frame_count = frame_count % 50

            rate.sleep()
