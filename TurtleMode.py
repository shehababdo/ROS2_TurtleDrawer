import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Empty
import math


class TurtleModeClass(Node):
    def __init__(self):
        super().__init__('TurtleMode')
        callback_group = ReentrantCallbackGroup()
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10,callback_group=callback_group)
        self.cmd_subscriber = self.create_subscription(Int16, 'command', self.cmd_callback, 10,callback_group=callback_group)

        self.pose = Pose()
        self.cmd = Twist()
        self.stop=0
    
    def pose_callback(self, data):
        self.pose = data

    def go_to_goal(self, goal):
     distance_to_goal = math.sqrt(pow((goal.linear.x - self.pose.x), 2) + pow((goal.linear.y - self.pose.y), 2))
     angle_to_goal = math.atan2(goal.linear.y - self.pose.y, goal.linear.x - self.pose.x)

     dist_tol = 0.1
     angle_tol = 0.01

     angle_error = angle_to_goal - self.pose.theta

     while distance_to_goal >= dist_tol:
            time = 4
            V_x = time*(goal.linear.x - self.pose.x)
            V_y = time*(goal.linear.y - self.pose.y)

            self.cmd.linear.x = V_x
            self.cmd.linear.y = V_y
            self.cmd.angular.z = 0.0

            self.cmd_vel_publisher.publish(self.cmd)
            distance_to_goal = math.sqrt(pow((goal.linear.x - self.pose.x), 2) + pow((goal.linear.y - self.pose.y), 2))
        #angle_error = angle_to_goal - self.pose.theta
    


    def draw_number_5(self):
        goal = Twist()
        init_x = self.pose.x
        init_y = self.pose.y
        size = 2.0

    # Draw the top arc of number 5
        

        x = init_x + size
        y = init_y 
        goal.linear.x = x
        goal.linear.y = y
        self.go_to_goal(goal)
        
        init_x = self.pose.x
        init_y = self.pose.y

        x=init_x-size
        y=init_y
        goal.linear.x = x
        goal.linear.y = y
        self.go_to_goal(goal)

        init_x = self.pose.x
        init_y = self.pose.y
        
        x = init_x 
        y = init_y - size/2
        goal.linear.x = x
        goal.linear.y = y
        self.go_to_goal(goal)
        
        init_x = self.pose.x
        init_y = self.pose.y
        
        theta=math.pi/2
        while theta>= -math.pi/2 and self.stop==0:
            x =  init_x+size*math.cos(theta)
            y = init_y/2+size*math.sin(theta)
            goal.linear.x = x
            goal.linear.y = y
            self.go_to_goal(goal)
            theta -= 0.01
            
    

    def flower(self):
        goal=Twist()
        init_x = self.pose.x
        init_y = self.pose.y
        self.angle = 0
        self.R = 2.0 
        while self.angle>=0 and self.stop==0:
            x =init_x+(self.R * math.cos(4 * self.angle)) * math.cos(self.angle)
            y = init_y+(self.R * math.cos(4 * self.angle)) * math.sin(self.angle)

            goal.linear.x = x
            goal.linear.y = y
            self.go_to_goal(goal)
            self.angle += 0.01

    def Lissajous_curve(self):
        goal=Twist()
        init_x = self.pose.x
        init_y = self.pose.y

        theta=0
        while theta<=8*math.pi and self.stop:
            x =  init_x+2*math.cos(3*theta)
            y = init_y+2*(math.sin(2*theta))
            goal.linear.x = x
            goal.linear.y = y
            self.go_to_goal(goal)
            theta += 0.01

    def reset_turtle(self):
        client = self.create_client(Empty, '/reset')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        request = Empty.Request()
        future = client.call_async(request)
       # rclpy.spin_until_future_complete(self, future)


    def cmd_callback(self, request):
        request = int(request.data)
        if request == 1:
            self.stop=0
            self.flower()
        elif request == 2:
            self.stop=0
            self.draw_number_5()
        elif request == 3:
            self.stop=1
            self.Lissajous_curve()
        elif request == 4:
            self.stop=1
            self.cmd.linear.x = 0.0
            self.cmd.linear.y = 0.0
            self.cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(self.cmd)
            self.reset_turtle()
        elif request == 5:
            self.stop =1
            self.msg.linear.x = 0.0
            self.msg.linear.y = 0.0 
            self.cmd_vel_publisher.publish(self.cmd)

    




    


def main(args=None):
    rclpy.init(args=args)
    node = TurtleModeClass()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()