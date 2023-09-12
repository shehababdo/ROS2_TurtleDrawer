import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


class PatternClass (Node):
    def __init__(self):
        super().__init__('PatternType')
        self.publisher_=self.create_publisher(Int16, 'command',10)
        self.timer= self.create_timer(0.1,self.timer_callback)


    def timer_callback(self):
        Type =Int16()
        Type.data=int(input('please enter the desigerd pattern number (1-flower pattern,2-number five pattern,3-Lissajous patter,4-Reset,5-Stop)'))

        self.publisher_.publish(Type)
    

def main(args=None):
 rclpy.init(args=args)
 node=PatternClass()
 rclpy.spin(node)
 node.destroy_node()
 rclpy.shutdown()


if __name__=='__main__':
   main()