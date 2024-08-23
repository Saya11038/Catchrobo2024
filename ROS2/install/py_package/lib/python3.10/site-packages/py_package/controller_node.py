import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from py_package.cybergear import Cybergear
# from std_msgs.msg import Float64




class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # JoyトピックをSubscribe
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        #Cybergear初期化
        self.motor_1 = Cybergear(253, 126)
        self.motor_2 = Cybergear(253, 127)

        self.motor_1.power_on()
        self.motor_2.power_on()

        self.motor_1.set_run_mode("location")
        self.motor_2.set_run_mode("location")

        self.motor_1.enable_motor()
        self.motor_2.enable_motor()

        self.motor_1.homing_mode()
        self.motor_2.homing_mode()

        self.motor_1.position_control(0.0, 2.0)
        self.motor_2.position_control(0.0, 2.0)

        self.axis = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.button = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


    def joy_callback(self, msg):
        # Axesの値を表示
        self.get_logger().info('Axes:')
        for i, axis in enumerate(msg.axes):
            self.get_logger().info(f'  Axis {i}: {axis}')
            self.axis[i] = axis

        # Buttonsの値を表示
        self.get_logger().info('Buttons:')
        for i, button in enumerate(msg.buttons):
            self.get_logger().info(f'  Button {i}: {button}')
            self.button[i] = button

        if self.axis[6] == 1.0:
            self.motor_1.position_control(3.0, 1.0)
            self.motor_2.position_control(3.0, 1.0)
        elif self.axis[6] == 0.0:
            self.motor_1.position_control(3.0, 0.0)
            self.motor_2.position_control(3.0, 0.0)
        elif self.axis[6] == -1.0:
            self.motor_1.position_control(-3.0, 1.0)
            self.motor_2.position_control(-3.0, 1.0)
            

#Axis 0,1:左スティック
#Axis 2:LT?　デフォルトで１
#Axis 3,4:右スティック
#Axis 5:RT  デフォルトで１
#Axis 6:左右　左→１，右→−１
#Axis 7:上下　上→１、下→−１

#Button 0:A 下
#Button 1:B 右
#Button 2:X 左
#Button 3:Y 上
#Button 4:LB
#Button 5:RB
#Button 6:Back
#Button 7:Start
#Button 8:logicool
        
def main(args=None):
    rclpy.init(args=args)
    sub_controller_node = ControllerNode()
    rclpy.spin(sub_controller_node)
    sub_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


