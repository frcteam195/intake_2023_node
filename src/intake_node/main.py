#!/usr/bin/env python3

import rospy

from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
from threading import Thread


from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotStatusHelperPy, Alliance, RobotMode, BufferedROSMsgHandlerPy
from ck_ros_msgs_node.msg import Intake_Control, Intake_Status


def publish_intake_1_link(roll_degrees : float, yaw_degrees: float, Pinched: bool):
    transform = Transform()
   # transform.linear.z = 0 # 
    if not Pinched:
        transform.angular.roll = math.radians(roll_degrees)
    else:
        transform.angular.roll = math.radians(15)

    transform.angular.yaw = math.radians(yaw_degrees)
    transform.linear.z = 0.4064


    transform_link = TransformLink("intake_1", "arm_extender")
    transform_link.set_transform(transform)
    transform_link.publish()

def publish_intake_2_link(roll_degrees : float, yaw_degrees: float, Pinched: bool):
    transform = Transform()
    # transform.linear.z = 0 # 
    if not Pinched:
        transform.angular.roll = math.radians(roll_degrees)
    else:
        transform.angular.roll = math.radians(-15)

    transform.angular.yaw = math.radians(yaw_degrees)
    transform.linear.z = 0.4064

    transform_link = TransformLink("intake_2", "arm_extender")
    transform_link.set_transform(transform)
    transform_link.publish()

def publish_arrow_1_link(roll_degrees : float, intake: float):
    transform = Transform()
    # transform.linear.z = 0 # 

    if intake > 0:
        transform.angular.roll = math.radians(roll_degrees)
    elif intake < 0:
        transform.angular.roll = math.radians(180-roll_degrees)
    else:
        transform.angular.roll = math.radians(90)

    transform.linear.z = 1.5

    transform_link = TransformLink("arrow_1", "arm_extender")
    transform_link.set_transform(transform)
    transform_link.publish()
    
def publish_arrow_2_link(roll_degrees : float, intake: float):
    transform = Transform()
    # transform.linear.z = 0 # 

    if intake > 0 :
        transform.angular.roll = math.radians(roll_degrees)
    elif intake < 0:
        transform.angular.roll = math.radians(180-roll_degrees)
    else:
        transform.angular.roll = math.radians(-90)

    transform.linear.z = 1.5

    transform_link = TransformLink("arrow_2", "arm_extender")
    transform_link.set_transform(transform)
    transform_link.publish()

def ros_func():
    global hmi_updates
    global robot_status

    control_subscriber = BufferedROSMsgHandlerPy(Intake_Control)
    control_subscriber.register_for_updates("IntakeControl")
    status_publisher = rospy.Publisher(
        name="IntakeStatus", data_class=Intake_Status, queue_size=50, tcp_nodelay=True)

    intakeRollerMotor = Motor("intake", MotorType.TalonFX)
    # wristRollerMotor = Motor("wrist", MotorType.TalonFX)

    pincherSolenoid = Solenoid("pincher", SolenoidType.SINGLE)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        if control_subscriber.get() is not None:
            intake_ctrl_msg : Intake_Control = control_subscriber.get()
            if robot_status.get_mode() == RobotMode.DISABLED:
                intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                pass
            else:
                if intake_ctrl_msg.rollers_intake:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 1.0, 0.0)
                    pass
                elif intake_ctrl_msg.rollers_outtake:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, -1.0, 0.0)
                    pass
                else:
                    intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                    pass

                if intake_ctrl_msg.pincher_solenoid_on:
                    pincherSolenoid.set(SolenoidState.ON)
                else:
                    pincherSolenoid.set(SolenoidState.OFF)
        
        publish_intake_1_link(45, 25, pincherSolenoid.get() == SolenoidState.ON)
        publish_intake_2_link(-45, 25, pincherSolenoid.get() == SolenoidState.ON)
        publish_arrow_1_link(45, 190)#intakeRollerMotor.get_sensor_velocity())
        publish_arrow_2_link(-45, 190)#intakeRollerMotor.get_sensor_velocity())

        status_message = Intake_Status()
        status_publisher.publish(status_message)

        rate.sleep()


def ros_main(node_name):
    rospy.init_node(node_name)
    register_for_robot_updates()

    intake_rectangle_1 = Cube("robot_parts", 4, "intake_1")
    intake_rectangle_1_transform = Transform()
    intake_rectangle_1_transform.linear.z = 0.2032 #12 in
    intake_rectangle_1.set_transform(intake_rectangle_1_transform)
    intake_rectangle_1.set_scale(Scale(0.0508, 0.0508, 0.4064)) #14 out (11.995, 16, 2) 
    intake_rectangle_1.set_color(Color(.7, .7, .7, 1.0))
    intake_rectangle_1.publish()

    intake_rectangle_2 = Cube("robot_parts", 5, "intake_2")
    intake_rectangle_2_transform = Transform()
    intake_rectangle_2_transform.linear.z = 0.2032 #12 in
    intake_rectangle_2.set_transform(intake_rectangle_2_transform)
    intake_rectangle_2.set_scale(Scale(0.0508, 0.0508, 0.4064)) #14 out (11.995, 16, 2) 
    intake_rectangle_2.set_color(Color(.7, .7, .7, 1.0))
    intake_rectangle_2.publish()

    arrow_rectangle_1 = Cube("arrow", 0, "arrow_1")
    arrow_rectangle_1_transform = Transform()
    arrow_rectangle_1_transform.linear.z = 0.2032 #12 in
    arrow_rectangle_1.set_transform(arrow_rectangle_1_transform)
    arrow_rectangle_1.set_scale(Scale(0.0508, 0.0508, 0.4064)) #14 out (11.995, 16, 2) 
    arrow_rectangle_1.set_color(Color(.7, .7, .7, 1.0))
    arrow_rectangle_1.publish()

    arrow_rectangle_2 = Cube("arrow", 1, "arrow_2")
    arrow_rectangle_2_transform = Transform()
    arrow_rectangle_2_transform.linear.z = 0.2032 #12 in
    arrow_rectangle_2.set_transform(arrow_rectangle_2_transform)
    arrow_rectangle_2.set_scale(Scale(0.0508, 0.0508, 0.4064)) #14 out (11.995, 16, 2) 
    arrow_rectangle_2.set_color(Color(.7, .7, .7, 1.0))
    arrow_rectangle_2.publish()
 
    t1 = Thread(target=ros_func)
    t1.start()

    rospy.spin()

    t1.join(5)
