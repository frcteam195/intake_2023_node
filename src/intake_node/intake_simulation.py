from ck_utilities_py_node.transform_links import *
from ck_utilities_py_node.rviz_shapes import *
import math

class IntakeSimulation:
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

    
    def publish_intake_1_link(self, Pinched: bool):
        transform = Transform()

        transform.angular.roll = math.radians(15) if Pinched else math.radians(45)
        

        transform.linear.z = 0.4064


        transform_link = TransformLink("intake_1", "wrist_link")
        transform_link.set_transform(transform)
        transform_link.publish()

    def publish_intake_2_link(self, Pinched: bool):
        transform = Transform()
    
        transform.angular.roll = math.radians(-15) if Pinched else math.radians(-45)
        
        transform.linear.z = 0.4064

        transform_link = TransformLink("intake_2", "wrist_link")
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

        transform_link = TransformLink("arrow", "wrist_link")
        transform_link.set_transform(transform)
        transform_link.publish()