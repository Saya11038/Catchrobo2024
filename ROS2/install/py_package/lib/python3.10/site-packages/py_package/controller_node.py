import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from py_package.cybergear import Cybergear
from std_msgs.msg import Int32
# from std_msgs.msg import Float64
import time
import math

esc_msg = Int32()

z_max = 0.0
z_min = -8.0

theta_per_rad = math.pi / 2 / 4.70  # rad/rad

r_per_rad = 16.0  # mm/rad

angle_max = 12.566


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

        self.esp32_publisher = self.create_publisher(
            Int32,
            'micro_ros_arduino_node_subscriber',
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

        self.esc_a = 0
        self.esc_b = 0
        self.esc_c = 0

        self.esc_a_before = 0
        self.esc_b_before = 0
        self.esc_c_before = 0

        self.x = 0.0
        self.y = 0.0

        self.new_x = 0.0
        self.new_y = 0.0

        self.r = 0.0
        self.theta = 0.0

        self.new_r = 0.0
        self.new_theta = 0.0

        self.diff_r = 0.0
        self.diff_theta = 0.0


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

            
        # Axis 0,1:左スティック  Axis 0 : 左が正、　Axis 1 : 上が正
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


        if self.calibration_num == 3:

            if not (self.axis[0] == 0.0 and self.axis[1] == 0.0):

                self.calc_xy()

                self.new_x = self.x - self.axis[0] * 10
                self.new_y = self.y + self.axis[1] * 10

                print(self.new_x, self.new_y)
                print("\n")
        
                self.calc_rtheta()

                # self.diff_r = abs(self.new_r - self.motor_r.angle)
                # self.diff_theta = abs(self.new_theta - self.motor_theta.angle)

                self.motor_r.position_control(self.new_r, 7.8)  # 4
                self.motor_theta.position_control(self.new_theta, 2.3)  # 1


        if self.button[1] - self.esc_a_before == 1:
            if self.esc_a == 1:
                self.esc_a = 0
                self.esc_a_before = 1
            else:
                self.esc_a = 1
                self.esc_a_before = 1
        else:
            self.esc_a_before = self.button[1]

        
        if self.button[2] - self.esc_b_before == 1:
            if self.esc_b == 1:
                self.esc_b = 0
                self.esc_b_before = 1
            else:
                self.esc_b = 1
                self.esc_b_before = 1
        else:
            self.esc_b_before = self.button[2]


        if self.button[3] - self.esc_c_before == 1:
            if self.esc_c == 1:
                self.esc_c = 0
                self.esc_c_before = 1
            else:
                self.esc_c = 1
                self.esc_c_before = 1
        else:
            self.esc_c_before = self.button[3]


        # if self.button[4] == 1 and self.motor_z.angle < z_max + 0.2:
        #     self.motor_z.speed_control(30.0, 4.0)
        # else:
        #     self.motor_z.speed_control(0.0, 2.0)

        # if self.button[5] == 1 and self.motor_z.angle > z_min:
        #     self.motor_z.speed_control(-8.0, 2.0)
        # else:
        #     self.motor_z.speed_control(0.0, 2.0)

        if self.button[4] == 1 and self.motor_z.angle < z_max:
            self.motor_z.position_control(z_max, -30.0)
        else:
            self.motor_z.position_control(z_max, 0.0)

        if self.button[5] == 1:
            self.motor_z.position_control(z_min, 8.0)
        else:
            self.motor_z.position_control(z_min, 0.0)

        if self.button[6] == 1:
            self.enable()

        if self.button[7] == 1:
            self.calibrate_all_motors()

        if self.button[8] == 1:
            self.shutdown()

        self.esp32_publish()


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
            self.motor_theta.position_control(4.70, 2.0)
            self.theta = 4.70 * theta_per_rad
            self.calibration_num = 1


    def calibration_z(self):

        self.motor_z.set_run_mode("speed")
        if self.sensor_data != 6:
            self.motor_z.speed_control(2.5, 2.0)
        else:
            self.motor_z.speed_control(0.0, 2.0)
            self.motor_z.set_run_mode("location")
            self.motor_z.homing_mode()
            self.motor_z.position_control(0.0, 2.0)
            # self.motor_z.set_run_mode("speed")
            self.calibration_num = 2

    
    def calibration_r(self):

        self.motor_r.set_run_mode("speed")
        if self.sensor_data != 4:
            self.motor_r.speed_control(-2.0, 2.0)
        else:
            self.motor_r.speed_control(0.0, 2.0)
            self.motor_r.set_run_mode("location")
            self.motor_r.homing_mode()
            self.motor_r.position_control(0.0, 2.0)
            time.sleep(1)
            self.motor_r.position_control(angle_max, 4.0)
            time.sleep(5)
            self.motor_r.homing_mode()
            self.motor_r.position_control(0.0, 2.0)
            self.r = angle_max * r_per_rad
            self.calibration_num = 3


    def esp32_publish(self):

        esc_msg.data = self.esc_c << 2 | self.esc_b << 1 | self.esc_a
        self.esp32_publisher.publish(esc_msg)

    
    def enable(self):

        self.motor_r.set_run_mode("location")
        self.motor_theta.set_run_mode("location")
        self.motor_z.set_run_mode("location")

        self.motor_r.enable_motor()
        self.motor_theta.enable_motor()
        self.motor_z.enable_motor()

        self.calibration_num = 0

    
    def shutdown(self):

        self.motor_r.stop_motor()
        self.motor_theta.stop_motor()
        self.motor_z.stop_motor()

    
    def calc_xy(self):

        self.motor_r.update_state()
        self.motor_theta.update_state()
        self.motor_z.update_state()

        self.x = - (angle_max + self.motor_r.angle) * r_per_rad * math.cos(self.motor_theta.angle * theta_per_rad)
        self.y = (angle_max + self.motor_r.angle) * r_per_rad * math.sin(self.motor_theta.angle * theta_per_rad)

        print(self.x, self.y)
        print("\n")

# 右　354.95mm  左　345.047
    
    def calc_rtheta(self):

        # if self.new_y < 0:
        #     if self.new_x < 100 and self.new_x > -100:
        #         self.new_x = self.x
        #         self.new_y = self.y

        self.new_r = math.sqrt(self.new_x * self.new_x + self.new_y * self.new_y)

        if self.new_x > 0 and self.new_y < 0:
            self.new_theta = math.atan2(- self.new_y, self.new_x) +  math.pi
        elif self.new_x == 0:
            self.new_theta = math.pi / 2
        else:
            self.new_theta = math.atan2(self.new_y, - self.new_x)

        # print(self.new_r, self.new_theta)
        # print("\n")

        print(-self.new_r * math.cos(self.new_theta), self.new_r * math.sin(self.new_theta))
        print("\n")

        self.new_r = (self.new_r - angle_max * r_per_rad) / r_per_rad
        self.new_theta = self.new_theta / theta_per_rad

        print(self.new_r, self.new_theta)
        print("\n")

        
def main(args=None):
    rclpy.init(args=args)
    sub_controller_node = ControllerNode()
    rclpy.spin(sub_controller_node)
    sub_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


