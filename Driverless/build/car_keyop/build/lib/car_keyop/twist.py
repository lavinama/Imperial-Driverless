from .base import run_operator

from geometry_msgs.msg import Twist, Vector3

def main():
    run_operator(
        make_publisher=lambda node: node.create_publisher(Twist, 'cmd_vel', 10),
        publish_callback=lambda pub, speed, steer: pub.publish(Twist(linear=Vector3(x=speed), angular=Vector3(z=steer)))
    )
