'''my node to drive
    john rader
    Created: 08/01/2024
    Last Edited: 10/22/2024
'''

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

class BrakeNode(Node):
    def __init__(self):
        super().__init__('brake_node')
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info('BrakeNode has been started.')

        self.LIDAR_LEN = 1080
        self.MID_INDEX = 540

        # modify these parameters for altered behavior
        self.WALL_BUFFER = 100 # determines how far out the closest lidar point extends outwards
        self.HORIZONTAL_CRASH_BUFFER_MIN = 0.30 # has 100% influence (vs driving straight) if it is less than 0.4
        self.HORIZONTAL_CRASH_BUFFER_MAX = 1.3 # has zero influence if it is past this point
        self.ANTI_CRASH_FORCE = 0.2
        self.ANTI_CRASH_INFLUENCE_MAX = 0.75
        self.SIDE_CRASH_INDEX_MIN = 250
        self.SIDE_CRASH_INDEX_MAX = 260

        self.SPEED_MIN = 3.0
        self.SPEED_MAX = 7.0

        self.SPEED_MIN_DIST = 3.5
        self.SPEED_MAX_DIST = 15

        self.left_memory = []
        self.right_memory = []
        self.MEMORY_LEN = 10
        self.MEMORY_DISTANCE_OFFSET = 0.1

    # this is where all the logic goes
    def scan_callback(self, msg):
        # using a ftg algorithm
        lidar = list(msg.ranges)[::-1]

        speed_out = self.SPEED_MIN
        dir_out = 0.0

        # avoid the closest lidar and the surrounding angles

        # this is so the car doesn't think that it can go in a direction because it doesn't know its own shape
        min_lidar_index = lidar.index(min(lidar))
        # set surrounding values to -1, they are too close to a wall to consider anyways
        # make sure you don't access out of bounds index
        max_avoid = min(self.LIDAR_LEN - 1, min_lidar_index + self.WALL_BUFFER)
        min_avoid = max(0, min_lidar_index - self.WALL_BUFFER)
        # set those values to -1
        lidar_with_avoid = lidar.copy()
        # repopulate indexes with -1
        lidar_with_avoid[min_avoid:max_avoid] = [-1] * (max_avoid - min_avoid)

        # now find the newest longest lidar value (shouldn't be super close to a wall)
        max_lidar_index = lidar_with_avoid.index(max(lidar_with_avoid))
        
        # calculate direction

        # calculate how much it needs to turn
        index_dist_from_mid = max_lidar_index - self.MID_INDEX
        dir_out = index_dist_from_mid / 100 # maybe try tuning this number?

        # if it wants to drive into a wall, stop it

        left_vals = lidar[self.SIDE_CRASH_INDEX_MIN : self.SIDE_CRASH_INDEX_MAX]
        right_vals = lidar[self.LIDAR_LEN - self.SIDE_CRASH_INDEX_MAX: self.LIDAR_LEN - self.SIDE_CRASH_INDEX_MIN]

        anti_crash_influence = 0
        anti_crash_dir = 1

        # move the car's turning right a variable amount
        # calculate the amount of influence the anti-crash algo should have
        a = self.HORIZONTAL_CRASH_BUFFER_MIN
        b = self.HORIZONTAL_CRASH_BUFFER_MAX
        x1 = min(left_vals)
        x2 = min(right_vals)
        anti_crash_dir = 1 if x1 < x2 else -1 # go left

        # calculate the amount of influence that each should have
        # (how likely from 0 to 1 is the car to crash on that side)
        linear_left = (-1 / (b - a)) * (x1 - b)
        linear_right = (-1 / (b - a)) * (x2 - b)
        linear_left = min(1, max(linear_left, 0))
        linear_right = min(1, max(linear_right, 0))

        # keep memory of left/right wall dist values
        if(len(self.left_memory) < self.MEMORY_LEN):
            self.left_memory.append(min(left_vals))
        else:
            self.left_memory.pop(0)
            self.left_memory.append(min(left_vals))

        if(len(self.right_memory) < self.MEMORY_LEN):
            self.right_memory.append(min(right_vals))
        else:
            self.right_memory.pop(0)
            self.right_memory.append(min(right_vals))

        adjusted_val = abs(linear_left - linear_right)
        anti_crash_influence = min(1, adjusted_val)
        
        # cap anti crash influence
        anti_crash_influence = min(anti_crash_influence, self.ANTI_CRASH_INFLUENCE_MAX)

        #dir_out = self.ANTI_CRASH_FORCE * anti_crash_influence * anti_crash_dir # test anti-crash
        
        # now calculate speed
        # linear from (x,y) = (speed_min_dist, speed_min)
        # to (x,y) = (speed_max_dist, speed_max)
        # speed_min when dist < speed_min_dist
        # speed_max when dist > speed_max_dist
        a = self.SPEED_MIN_DIST
        b = self.SPEED_MAX_DIST
        n = self.SPEED_MAX
        m = self.SPEED_MIN
        x = lidar[max_lidar_index]
        speed_val = (-(x-b)/(b-a))*(m-n) + n
        speed_val = min(self.SPEED_MAX, max(self.SPEED_MIN, speed_val))
        speed_out = speed_val

        # if the average of the newer values is less than that of older values
        # then the distance is decreasing
        left_old_avg = sum(left_vals[:self.MEMORY_LEN // 2]) // (self.MEMORY_LEN // 2)
        right_old_avg = sum(right_vals[:self.MEMORY_LEN // 2]) // (self.MEMORY_LEN // 2)
        left_new_avg = sum(left_vals[self.MEMORY_LEN // 2:]) // (self.MEMORY_LEN // 2)
        right_new_avg = sum(right_vals[self.MEMORY_LEN // 2:]) // (self.MEMORY_LEN // 2)

        if  (left_new_avg > left_old_avg + self.MEMORY_DISTANCE_OFFSET and anti_crash_dir == 1) or \
            (right_new_avg > right_old_avg + self.MEMORY_DISTANCE_OFFSET and anti_crash_dir == -1):
            # the car likely isn't crashing, reduce speed wobbling
            dir = "left" if anti_crash_dir == -1 else "right"
            #print(f"increasing distance, dir = {dir}") 
            anti_crash_influence *= 0.2
        else:
            #print("decreasing distance")
            pass
        
        # lazy check to sometimes stop it from crashing
        if lidar[self.MID_INDEX] < 2:
            speed_out = 0.75

        dir_out = self.ANTI_CRASH_FORCE * anti_crash_influence * anti_crash_dir + (1 - anti_crash_influence) * dir_out

        #print(f"speed_out: {speed_out}")
        #print(f"max_dist: {x}")
        speed_out = float(speed_out)
        dir_out = -float(dir_out)
        self.publish_ackermann_drive(speed_out, dir_out)

    def odom_callback(self, msg):
        self.get_logger().info('Received Odometry data')
        # Process Odometry data here

    def publish_ackermann_drive(self, speed, steering_angle):
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header = Header()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.drive.speed = speed
        ackermann_msg.drive.steering_angle = steering_angle

        self.ackermann_publisher.publish(ackermann_msg)
        #self.get_logger().info(f'Published AckermannDriveStamped message: speed={speed}, steering_angle={steering_angle}')

def main(args=None):
    rclpy.init(args=args)
    node = BrakeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
