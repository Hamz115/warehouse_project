#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time

class ElevatorController:
    def __init__(self, parent_node):
        self.node = parent_node
        self.lift_publisher = self.node.create_publisher(String, '/elevator_up', 10)
        self.lower_publisher = self.node.create_publisher(String, '/elevator_down', 10)

    def lift(self):
        self.node.get_logger().info("Attempting to raise elevator")
        msg = String()
        msg.data = "up"
        self.lift_publisher.publish(msg)
        time.sleep(6)  # Wait for elevator to complete its motion
        self.node.get_logger().info("Elevator raise command sent and completed")
        return True

    def lower(self):
        self.node.get_logger().info("Lowering elevator")
        msg = String()
        msg.data = "down"
        self.lower_publisher.publish(msg)
        time.sleep(6)  
        self.node.get_logger().info("Elevator lower command sent and completed")

class FootprintManager:
    def __init__(self, parent_node):
        self.node = parent_node

    def update_footprint_with_shelf(self):
        self.node.get_logger().info("Updating footprint for robot with shelf")
        

    def reset_footprint(self):
        self.node.get_logger().info("Resetting robot footprint")
        

class ShelfAttacher:
    def __init__(self, parent_node):
        self.node = parent_node
        

    def attach(self):
        self.node.get_logger().info("Attaching to shelf")
        # Implement shelf attachment logic
        time.sleep(2)  # Simulate attachment time
        self.node.get_logger().info("Shelf attachment mechanism activated")
        return True  

class WarehouseRobotController(Node):
    def __init__(self):
        super().__init__('warehouse_robot_controller')
        self.navigator = BasicNavigator()
        self.elevator_controller = ElevatorController(self)
        self.footprint_manager = FootprintManager(self)
        self.shelf_attacher = ShelfAttacher(self)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.distance_to_shelf = float('inf')
        self.initialize_robot()

    def initialize_robot(self):
        self.get_logger().info("Initializing robot...")
        initial_pose = self.create_pose('init_position')
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Robot initialized successfully")

    def laser_callback(self, msg):
        # Assuming the shelf is directly in front of the robot
        self.distance_to_shelf = min(msg.ranges)

    def create_pose(self, position_key):
        positions = {
            "init_position": [-0.29141, -0.463275, -0.106878, 0.994272],
            "loading_position": [3.95511, -1.32686, -0.837662, 0.54619],
            "pre_shipping_position": [1.04646, -0.864842, 0.957607, 0.288078],
            "shipping_position": [1.4481, 0.38586, 0.678121, 0.73495]
        }
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.z, pose.pose.orientation.w = positions[position_key]
        return pose

    def execute_mission(self):
        try:
            self.go_to_loading_area()
            if self.final_approach():
                if self.attach_shelf():
                    self.move_backwards()
                    self.transport_to_shipping()
                    self.unload_shelf()
                    self.return_to_base()
                else:
                    raise RuntimeError("Failed to attach shelf")
            else:
                raise RuntimeError("Failed to perform final approach")
        except Exception as e:
            self.get_logger().error(f"Mission failed: {str(e)}")
            self.emergency_stop()
            self.return_to_base()

    def go_to_loading_area(self):
        target_pose = self.create_pose('loading_position')
        self.navigate_to(target_pose, 'loading area')
    
        # Additional forward movement
        self.get_logger().info("Performing slight forward movement")
        twist = Twist()
        twist.linear.x = 0.1  # 0.1 m/s forward
    
        # Move forward for 2 seconds
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 3.0:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)
    
        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Slight forward movement completed")

    def emergency_stop(self):
        self.get_logger().warn("Performing emergency stop")
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)

    def final_approach(self):
        self.get_logger().info("Performing final approach to shelf")
        approach_distance = 1.5  # meters
        approach_speed = 0.2  # m/s
        approach_duration = approach_distance / approach_speed

        twist = Twist()
        twist.linear.x = approach_speed

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < approach_duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)

        if self.check_shelf_position():
            self.get_logger().info("Final approach successful")
            return True
        else:
            self.get_logger().error("Final approach failed")
            return False

    def check_shelf_position(self):
        self.get_logger().info("Checking shelf position")
        
        return True

    def attach_shelf(self):
        self.get_logger().info("Initiating shelf attachment procedure")
        
        if not self.fine_tune_position():
            self.get_logger().error("Failed to fine-tune position for shelf attachment")
            return False

        for attempt in range(3): 
            self.get_logger().info(f"Attachment attempt {attempt + 1}")
            
            if self.shelf_attacher.attach():
                self.get_logger().info("Shelf attachment successful")
                
                if self.elevator_controller.lift():
                    self.get_logger().info("Elevator raised successfully")
                    self.footprint_manager.update_footprint_with_shelf()
                    
                    if self.verify_attachment():
                        self.get_logger().info("Shelf attachment verified")
                        return True
                    else:
                        self.get_logger().warn("Shelf attachment verification failed")
                else:
                    self.get_logger().error("Failed to raise elevator")
            else:
                self.get_logger().warn("Shelf attachment failed")
            
           
            self.elevator_controller.lower()
            time.sleep(1)  
        
        self.get_logger().error("Failed to attach shelf after multiple attempts")
        return False

    def fine_tune_position(self):
        self.get_logger().info("Fine-tuning position for shelf attachment")
        # Implement fine positioning logic here
        return True

    def verify_attachment(self):
        self.get_logger().info("Verifying shelf attachment")
        # Implement verification logic here
        return True

    def move_backwards(self):
        self.get_logger().info("Moving backwards to create maneuvering space")
        backward_distance = 1.5  # meters
        backward_speed = 0.2  # m/s
        backward_duration = backward_distance / backward_speed

        twist = Twist()
        twist.linear.x = -backward_speed

        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < backward_duration:
            self.cmd_vel_publisher.publish(twist)
            time.sleep(0.1)

        # Stop the robot
        twist.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info("Backward movement completed")

    def transport_to_shipping(self):
        self.get_logger().info("Transporting shelf to shipping area")
        
        # navigate to the pre-shipping position
        pre_shipping_pose = self.create_pose('pre_shipping_position')
        self.navigate_to(pre_shipping_pose, 'pre-shipping waypoint')
        
        # navigate to the final shipping position
        shipping_pose = self.create_pose('shipping_position')
        self.navigate_to(shipping_pose, 'shipping area')


    def unload_shelf(self):
        self.get_logger().info("Unloading shelf")
        self.elevator_controller.lower()
        self.footprint_manager.reset_footprint()
        self.move_backwards()  # Move backwards after unloading as well

    def return_to_base(self):
        base_pose = self.create_pose('init_position')
        self.navigate_to(base_pose, 'base')

    def navigate_to(self, pose, destination_name):
        self.navigator.goToPose(pose)
        
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and hasattr(feedback, 'estimated_time_remaining'):
                estimated_time = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                self.get_logger().info(f'ETA to {destination_name}: {estimated_time:.0f} seconds')
            time.sleep(0.5)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'Reached {destination_name}')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn(f'Navigation to {destination_name} was canceled')
        elif result == TaskResult.FAILED:
            self.get_logger().error(f'Failed to reach {destination_name}')
            raise RuntimeError(f'Navigation failed')
    
    def emergency_stop(self):
        self.get_logger().warn("Performing emergency stop")
        twist = Twist()
        self.cmd_vel_publisher.publish(twist)


def main():
    rclpy.init()
    controller = WarehouseRobotController()
    try:
        controller.execute_mission()
    except Exception as e:
        controller.get_logger().error(f"Mission failed: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()