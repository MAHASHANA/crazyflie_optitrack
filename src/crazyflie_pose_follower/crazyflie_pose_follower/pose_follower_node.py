# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
import cflib.crtp
import time
import threading
import math

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Reusable helper

def calculate_distance(p1, p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2 + (p2[2]-p1[2])**2)

def generate_descent_sequence(x, y, start_height, end_height, initial_step_size=0.1, min_step_size=0.01, decay_rate=0.95):
    descent_sequence = []
    current_height = start_height
    step_size = initial_step_size
    while current_height > end_height:
        descent_sequence.append((x, y, current_height))
        step_size = max(step_size * decay_rate, min_step_size)
        current_height = max(current_height - step_size, end_height)
    descent_sequence.append((x, y, end_height))
    return descent_sequence


def get_fixed_uav_sequences():
    uav_sequences = []
    z = 1.5
    cycle_1 = [
        (0.3, 0.3, z),
        (0.1, 1.1, z),
        # (-0.6, 0.4, z),
        # (0.2, 0.6, z),
    ]
    cycle_2 = [
        (-0.3, 0.3, z),
        (-0.1, 1.1, z),
        (0.6, 0.4, z),
        (-0.2, 0.6, z),
    ]

    uav_sequences.append(cycle_1)
    uav_sequences.append(cycle_2)
    return uav_sequences

class CrazyfliePoseFollower(Node):
    def __init__(self, cf):
        super().__init__('crazyflie_pose_follower')

        self.cf = cf
        self.current_pose = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

        self.subscription = self.create_subscription(
            PoseStamped,
            '/Robot_1/pose',
            self.pose_callback,
            10
        )

        # self.goal_subscription = self.create_subscription(
        #     PoseStamped,
        #     '/crazyflie/last_waypoint',
        #     self.goal_callback,
        #     10
        # )

        self.takeoff_subscription = self.create_subscription(
            String,
            '/crazyflie/takeoff',
            self.takeoff_callback,
            10
        )
        self.landing_pose_subscription = None

        self.last_waypoint = None
        self.flight_sequence = []
        self.takeoff_received = False
        self.is_flying = False
        self.current_cycle_index = 0
        self.predefined_flight_cycles = get_fixed_uav_sequences()
        self.get_logger().info("CrazyfliePoseFollower initialized.")

    def pose_callback(self, msg):
        with self.lock:
            self.current_pose = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )

    def takeoff_callback(self, msg):
        if msg.data.lower().strip() == "start":
            if not self.takeoff_received:
                self.takeoff_received = True
                self.get_logger().info("Takeoff signal received.")
                threading.Thread(target=self.execute_flight_sequence).start()

        
    def goal_callback(self, msg):
        if self.last_waypoint is None:
            landing_x = msg.pose.position.x
            landing_y = msg.pose.position.y
            fixed_z = 1.5
            self.last_waypoint = (landing_x, landing_y, fixed_z)
            self.get_logger().info(f"Captured /Robot_2/pose x,y with fixed z=1.5 as landing waypoint: {self.last_waypoint}")
            # self.predefined_flight_cycles[self.current_cycle_index].append(self.last_waypoint)
            # self.landing_pose_subscription.destroy()
        # self.last_waypoint = (
        #     msg.pose.position.x,
        #     msg.pose.position.y,
        #     msg.pose.position.z
        # )
        # self.get_logger().info(f"Received landing waypoint: {self.last_waypoint}")
        # self.predefined_flight_cycles[self.current_cycle_index].append(self.last_waypoint)
        # self.landing_pose_subscription.destroy()

    def execute_flight_sequence(self):
        commander = self.cf.high_level_commander

        self.is_flying = True
        commander.takeoff(0.7, 3.0)
        time.sleep(3.0)
        fixed_cycle = list(self.predefined_flight_cycles[self.current_cycle_index])

        for idx, (x, y, z) in enumerate(fixed_cycle):
            if self.stop_event.is_set():
                self.get_logger().warn("Stop signal received. Landing now.")
                break
            commander.go_to(x, y, z, 0.0, 2.0)
            self.get_logger().info(f"Flying to waypoint {idx+1}: ({x}, {y}, {z})")
            start_time = time.time()
            timeout = 10.0      # increase or decrease this timeout to balance hover time
            while time.time() - start_time < timeout:
                with self.lock:
                    current = self.current_pose
                if current:
                    dist = calculate_distance(current, (x, y, z))
                    if dist < 0.15:
                        self.get_logger().info(f"Reached waypoint {idx+1}.")
                        break
                time.sleep(0.2)

        # self.get_logger().info("Waiting for landing waypoint on /crazyflie/last_waypoint...")
        self.get_logger().info("Subscribing once to /Robot_2/pose for landing position...")
        self.landing_pose_subscription = self.create_subscription(
            PoseStamped,
            '/Robot_2/pose',
            self.goal_callback,
            10
        )

        while self.last_waypoint is None:
            time.sleep(0.1)

        # fixed_cycle.append(self.last_waypoint)

        descent_sequence = generate_descent_sequence(
            x=self.last_waypoint[0],
            y=self.last_waypoint[1],
            start_height=fixed_cycle[-1][2],
            end_height=0.3
        )

        for idx, (x, y, z) in enumerate(descent_sequence):
            if self.stop_event.is_set():
                break
            commander.go_to(x, y, z, 0.0, 2.0)
            self.get_logger().info(f"Descent step {idx+1}: ({x}, {y}, {z})")
            time.sleep(0.3)

        commander.land(0.3, 3.0)
        time.sleep(3.0)
        commander.stop()

        self.get_logger().info(f"Landing complete. Cycle {self.current_cycle_index + 1} done.")
        self.current_cycle_index += 1
        self.is_flying = False
        self.takeoff_received = False
        self.flight_sequence = []
        self.last_waypoint = None


def main(args=None):
    rclpy.init(args=args)
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        node = CrazyfliePoseFollower(cf=scf.cf)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()
