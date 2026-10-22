"""New simulation-only ROS topics. Not assumed Pinky manufacturer interfaces."""
import math
import os
import time
from .traffic import VelocityWatchdog


class RosLoopback:
    def __init__(self, app, config):
        # The launcher sets these before ROS libraries are loaded.
        if os.environ.get('ROS_DOMAIN_ID') != str(config['simulation']['ros_domain_id']):
            raise RuntimeError('Use the isolated simulation domain from layout config')
        if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
            raise RuntimeError('Local loopback test requires ROS_LOCALHOST_ONLY=1')
        import omni.kit.app
        import carb.tokens
        from pathlib import Path
        carb.tokens.get_tokens_interface().set_value('omni_cache',str(Path(__file__).resolve().parents[1]/'.runtime/kit'))
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate('isaacsim.ros2.bridge', True)
        for _ in range(5):
            app.update()
        import rclpy
        from rclpy.context import Context
        from geometry_msgs.msg import Twist, TransformStamped
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        self.rclpy, self.Twist = rclpy, Twist
        self.Odometry, self.Clock = Odometry, Clock
        self.JointState, self.TFMessage, self.TransformStamped = JointState, TFMessage, TransformStamped
        self.context = Context()
        rclpy.init(context=self.context)
        ns = config['simulation']['namespace']
        self.node = rclpy.create_node('sim_robot',namespace=ns,context=self.context)
        self.client = rclpy.create_node('smoke_client',namespace=ns,context=self.context)
        self.watchdog = VelocityWatchdog(config['simulation']['command_timeout'])
        self.received = 0
        self.counts = {'ground_truth_odom':0,'clock':0,'joint_states':0,'tf':0}
        self.subscription = self.node.create_subscription(Twist,'cmd_vel',self.on_command,10)
        self.publisher = self.client.create_publisher(Twist,'cmd_vel',10)
        self.pubs, self.subs = {}, []
        for key, topic, msg_type in [('ground_truth_odom','ground_truth/odom',Odometry),
                                     ('clock','clock',Clock),('joint_states','joint_states',JointState),
                                     ('tf','ground_truth/tf',TFMessage)]:
            self.pubs[key] = self.node.create_publisher(msg_type,topic,10)
            self.subs.append(self.client.create_subscription(msg_type,topic,lambda msg,k=key:self.count(k),10))
        deadline = time.monotonic()+3
        while self.publisher.get_subscription_count() < 1 and time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=.02)
            rclpy.spin_once(self.client,timeout_sec=.02)
        if self.publisher.get_subscription_count() < 1:
            self.close()
            raise RuntimeError('ROS command endpoints did not discover each other')

    def count(self, key):
        self.counts[key] += 1

    def on_command(self, msg):
        self.received += 1
        self.watchdog.receive(msg.linear.x,msg.angular.z,time.monotonic())

    def exchange(self, v, w, pose, joints, names, sim_time):
        command = self.Twist()
        command.linear.x, command.angular.z = float(v),float(w)
        self.publisher.publish(command)
        for _ in range(2):
            self.rclpy.spin_once(self.node,timeout_sec=.001)
        clock = self.Clock()
        clock.clock.sec = int(sim_time)
        clock.clock.nanosec = int((sim_time-int(sim_time))*1e9)
        self.pubs['clock'].publish(clock)
        odom = self.Odometry()
        odom.header.stamp = clock.clock
        odom.header.frame_id = 'sim_world'
        odom.child_frame_id = 'pinky_1/base_link'
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z = pose[:3]
        odom.pose.pose.orientation.w = math.cos(pose[3]/2)
        odom.pose.pose.orientation.z = math.sin(pose[3]/2)
        self.pubs['ground_truth_odom'].publish(odom)
        joint = self.JointState()
        joint.header.stamp = clock.clock
        joint.name, joint.position = names,[float(v) for v in joints]
        self.pubs['joint_states'].publish(joint)
        tf = self.TransformStamped()
        tf.header,tf.child_frame_id = odom.header,odom.child_frame_id
        tf.transform.translation.x,tf.transform.translation.y,tf.transform.translation.z = pose[:3]
        tf.transform.rotation = odom.pose.pose.orientation
        self.pubs['tf'].publish(self.TFMessage(transforms=[tf]))
        for _ in range(5):
            self.rclpy.spin_once(self.client,timeout_sec=0)
        return self.watchdog.command(time.monotonic())

    def finish_test(self):
        # Drain the last commands, then verify an actual receiver-time gap.
        for _ in range(10):
            self.rclpy.spin_once(self.node,timeout_sec=.001)
            self.rclpy.spin_once(self.client,timeout_sec=.001)
        self.watchdog.receive(.1,0,time.monotonic())
        time.sleep(self.watchdog.timeout+.03)
        stale_stops = self.watchdog.command(time.monotonic()) == (0.0,0.0)
        return {'commands_received':self.received,'messages_received':self.counts,
                'watchdog_expiry':stale_stops,'passed':self.received>20 and all(self.counts.values()) and stale_stops,
                'scope':'same-process separate ROS nodes using RMW; no external Ubuntu connection, lidar or Nav2',
                'rmw':os.environ.get('RMW_IMPLEMENTATION'),'domain':os.environ.get('ROS_DOMAIN_ID')}

    def close(self):
        self.node.destroy_node()
        self.client.destroy_node()
        self.rclpy.shutdown(context=self.context)
