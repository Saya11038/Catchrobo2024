import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from py_package.cybergear import Cybergear
from std_msgs.msg import Int32
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

        # ESP32からのトピックをSubscribe
        self.esp32_subscription = self.create_subscription(
            Int32,  # ESP32でpublishされたデータの型
            'micro_ros_arduino_node_publisher',  # ESP32のトピック名
            self.esp32_callback,  # コールバック関数
            10
        )

        #Cybergear初期化
        self.motor_r = Cybergear(253, 127)
        self.motor_theta = Cybergear(253, 126)
        self.motor_z = Cybergear(253, 125)

        # r → 前向き正
        # theta → 右向き正
        # z → 上向き正

        self.motor_r.power_on()
        self.motor_theta.power_on()
        self.motor_z.power_on()

        self.motor_r.set_run_mode("location")
        self.motor_theta.set_run_mode("location")
        self.motor_z.set_run_mode("location")

        self.motor_r.enable_motor()
        self.motor_theta.enable_motor()
        self.motor_z.enable_motor()

        self.motor_r.homing_mode()
        self.motor_theta.homing_mode()
        self.motor_z.homing_mode()

        self.motor_r.position_control(0.0, 2.0)
        self.motor_theta.position_control(0.0, 2.0)
        self.motor_z.position_control(0.0, 2.0)

        self.axis = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
        self.button = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.sensor_data = 0
        self.calibration_num = 0


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
            self.motor_r.position_control(3.0, 1.0)
            self.motor_theta.position_control(3.0, 1.0)
        elif self.axis[6] == 0.0:
            self.motor_r.position_control(3.0, 0.0)
            self.motor_theta.position_control(3.0, 0.0)
        elif self.axis[6] == -1.0:
            self.motor_r.position_control(-3.0, 1.0)
            self.motor_theta.position_control(-3.0, 1.0)
            
        # Axis 0,1:左スティック
        # Axis 2:LT?　デフォルトで１
        # Axis 3,4:右スティック
        # Axis 5:RT  デフォルトで１
        # Axis 6:左右　左→１，右→−１
        # Axis 7:上下　上→１、下→−１

        # Button 0:A 下
        # Button 1:B 右
        # Button 2:X 左
        # Button 3:Y 上
        # Button 4:LB
        # Button 5:RB
        # Button 6:Back
        # Button 7:Start
        # Button 8:logicool

        if self.button[7] == 1:
            self.calibrate_all_motors()

        if self.button[8] == 1:
            self.shutdown()


    # ESP32からのデータを受け取るコールバック関数
    def esp32_callback(self, msg):
        self.get_logger().info(f'ESP32からのデータを受信: {msg.data}')
        # ESP32からのデータを使って何かの処理を行う
        self.sensor_data = msg.data


    def calibrate_all_motors(self):
        if self.calibration_num == 0:
            self.calibration_theta()
        elif self.calibration_num == 1:
            self.calibration_z()
        elif self.calibration_num == 2:
            self.calibration_r()

    
    def calibration_theta(self):

        self.motor_theta.set_run_mode("speed")
        if self.sensor_data != 3:
            self.motor_theta.speed_control(0.5, 2.0)
        else:
            self.motor_theta.speed_control(0.0, 2.0)
            self.motor_theta.set_run_mode("location")
            self.motor_theta.homing_mode()
            self.motor_theta.position_control(0.0, 2.0)
            self.calibration_num = 1


    def calibration_z(self):

        self.motor_z.set_run_mode("speed")
        if self.sensor_data != 1:
            self.motor_z.speed_control(1.0, 2.0)
        else:
            self.motor_z.speed_control(0.0, 2.0)
            self.motor_z.set_run_mode("location")
            self.motor_z.homing_mode()
            self.motor_z.position_control(0.0, 2.0)
            self.calibration_num = 2

    
    def calibration_r(self):

        self.motor_r.set_run_mode("speed")
        if self.sensor_data != 0:
            self.motor_r.speed_control(-1.0, 2.0)
        else:
            self.motor_r.speed_control(0.0, 2.0)
            self.motor_r.set_run_mode("location")
            self.motor_r.homing_mode()
            self.motor_r.position_control(0.0, 2.0)
            self.calibration_num = 3

    
    def shutdown(self):

        self.motor_r.stop_motor()
        self.motor_theta.stop_motor()
        self.motor_z.stop_motor()

        
def main(args=None):
    rclpy.init(args=args)
    sub_controller_node = ControllerNode()
    rclpy.spin(sub_controller_node)
    sub_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


