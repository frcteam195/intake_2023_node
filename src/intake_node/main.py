#!/usr/bin/env python3

from threading import Thread
import rospy
from ck_ros_msgs_node.msg import Intake_Control, Intake_Status
from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *

from ck_utilities_py_node.motor import *
from ck_utilities_py_node.solenoid import *

from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import RobotMode, BufferedROSMsgHandlerPy


class IntakeNode():

    def __init__(self) -> None:

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

        arrow = Arrow("arrow", 2, "arrow")
        arrow_transform = Transform()
        arrow.set_transform(arrow_transform)
        arrow.set_scale(Scale(0.5, 0.1, 0.1)) #14 out (11.995, 16, 2)
        arrow.set_color(Color(0.949, 0.875, 0.027, 1.0)) 
        arrow.publish()
        


        self.control_subscriber = BufferedROSMsgHandlerPy(Intake_Control)
        self.control_subscriber.register_for_updates("IntakeControl")
        self.status_publisher = rospy.Publisher(
            name="IntakeStatus", data_class=Intake_Status, queue_size=50, tcp_nodelay=True)
    
        register_for_robot_updates()
        t1 = Thread(target=self.loop)
        t1.start()

        rospy.spin()

        t1.join(5)
    
    def publish_intake_1_link(self, yaw_degrees: float, Pinched: bool):
        transform = Transform()
  
        transform.angular.roll = math.radians(15) if Pinched else math.radians(45)
           

        transform.angular.yaw = math.radians(yaw_degrees)
        transform.linear.z = 0.4064


        transform_link = TransformLink("intake_1", "arm_extender")
        transform_link.set_transform(transform)
        transform_link.publish()

    def publish_intake_2_link(self, yaw_degrees: float, Pinched: bool):
        transform = Transform()
      
        transform.angular.roll = math.radians(-15) if Pinched else math.radians(-45)
           
        transform.angular.yaw = math.radians(yaw_degrees)
        transform.linear.z = 0.4064

        transform_link = TransformLink("intake_2", "arm_extender")
        transform_link.set_transform(transform)
        transform_link.publish()
    
    def publish_arrow_link(self, pitch_degrees : float, intake: float):
        transform = Transform()
        
        if intake > 0 :
            transform.angular.pitch = math.radians(pitch_degrees)
            transform.linear.z = 1.5
        elif intake < 0:
            transform.angular.pitch = math.radians(-pitch_degrees)
            transform.linear.z = 0.9
        else:
            transform.linear.z = 100

       

        transform_link = TransformLink("arrow", "arm_extender")
        transform_link.set_transform(transform)
        transform_link.publish()

    def loop(self) -> None:

        self.intakeRollerMotor = Motor("intake", MotorType.TalonFX)

        self.pincherSolenoid = Solenoid("pincher", SolenoidType.SINGLE)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.control_subscriber.get() is not None:
                intake_ctrl_msg : Intake_Control = self.control_subscriber.get()
                if robot_status.get_mode() == RobotMode.DISABLED:
                    self.intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                    
                else:
                    if intake_ctrl_msg.rollers_intake:
                        self.intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 1.0, 0.0)
                        
                    elif intake_ctrl_msg.rollers_outtake:
                        self.intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, -1.0, 0.0)
                        
                    else:
                        self.intakeRollerMotor.set(ControlMode.PERCENT_OUTPUT, 0.0, 0.0)
                        

                    if intake_ctrl_msg.pincher_solenoid_on:
                       self.pincherSolenoid.set(SolenoidState.ON)
                    else:
                        self.pincherSolenoid.set(SolenoidState.OFF)
            
            
            
                #self.publish_intake_1_link(intake_ctrl_msg.wrist_twist, self.pincherSolenoid.get() == SolenoidState.ON)
                #self.publish_intake_\_link(intake_ctrl_msg.wrist_twist, self.pincherSolenoid.get() == SolenoidState.ON)
                self.publish_arrow_link(90, self.intakeRollerMotor.get_sensor_velocity())
            
            status_message = Intake_Status()
            self.status_publisher.publish(status_message)

            rate.sleep()


    
