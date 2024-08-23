import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import struct
import math
from py_package.cybergear import Cybergear
# from std_msgs.msg import Float64


# frame_head = "4154" #hex_str
# frame_tail = "0d0a" #hex_str

# index_angle = "1670"
# index_speed = "1770"
# index_speed_mode = "0a70"
# index_max_current = "1870"

# index_dict = {
#     "run_mode" : "0570",
#     "iq_ref" : "0670",
#     "id_ref" : "0770",
#     "spd_ref" : "0a70",
#     "limit_torque" : "0b70",
#     "cur_kp" : "1070",
#     "cur_ki" : "1170",
#     "cur_filt_gain" : "1470",
#     "loc_ref" : "1670",
#     "limit_spd" : "1770",
#     "limit_cur" : "1870",
#     "mechPos" : "1970",
#     "iqf" : "1a70",
#     "mechVel" : "1b70",
#     "VBUS" : "1c70",
#     "rotation" : "1d70",
#     "loc_kp" : "1e70",
#     "spd_kp" : "1f70",
#     "spd_ki" : "2070"
# }

# run_mode = {
#     "motion_control" : 0,
#     "location" : 1,
#     "speed" : 2,
#     "current" : 3
# }


# master_CANID = 0b0000000011111101 #bin_num
# motor_CANID = 0b01111111 #bin_num

# frame_enable_motor = 0b00000011
# frame_stop_motor = 0b00000100
# frame_read_param = 0b00010001
# frame_write_param = 0b00010010
# frame_homing_mode = 0b00000110
# frame_motion_control = 0b00000001
# frame_power_on = 0b00010011
# frame_get_device = 0b00000000


# #入力された整数値を2進数の数値に変える関数
# def int_to_bin(num_deci):

#     num = 0
#     num_bin = 0b0

#     while num_deci > 1:
#         if num_deci % 2 == 0:
#             num_deci = int(num_deci / 2)
#             num_bin = num_bin | 0b0 << num
#         else:
#             num_deci = int(num_deci / 2)
#             num_bin = num_bin | 0b1 << num
#         num += 1

#     if num_deci == 0:
#         num_bin = num_bin | 0b0 << num
#     elif num_deci == 1:
#         num_bin = num_bin | 0b1 << num

#     return num_bin


# #小数の小数部分を23bitの２進数の数値に変換
# def decimal_to_binary_fraction(decimal_fraction):
#     i = 0
#     binary_fraction = 0b0
#     while decimal_fraction != 0:
#         decimal_fraction *= 2
#         if decimal_fraction >= 1:
#             binary_fraction = binary_fraction << 1 | 0b1
#             decimal_fraction -= 1
#         else:
#             binary_fraction = binary_fraction << 1 | 0b0
#         i += 1
#         if i >= 23:  # 23ビットまで丸める
#             break
#     return binary_fraction


# #16進数の数字を2文字セットで逆向きにする
# def reverse_hex(hex_string):
#      # 文字列を2文字ずつに分割してリストに格納
#     pairs = [hex_string[i:i+2] for i in range(0, len(hex_string), 2)] 
#     # 逆順に並べ替えて結合
#     result = ''.join(pairs[::-1])  
#     return result


# def linear_mapping(data, min_num = 0.0, max_num = 65535.0, min_data = 0.0, max_data = 65535.0):
#     return (data - min_data + 1.0) / (max_data - min_data + 1.0) * (max_num - min_num) + min_num


# def hex_to_float(hex_str):
#     # 16進数文字列をバイト列に変換
#     byte_array = bytes.fromhex(hex_str)    
#     # バイト列を浮動小数点数に変換
#     float_num = struct.unpack('!f', byte_array)[0]
#     return float_num



# class Cybergear:


#     def __init__(self, master_can, motor_can):
#         self.master = int_to_bin(master_can)
#         self.motor = int_to_bin(motor_can)
#         self.angle = 0.0

#         #シリアルポートとボーレートを設定
#         self.ser = serial.Serial('/dev/ttyUSB0', 921600, timeout = 2.0)


#     def enable_motor(self):
        
#         bin_num = frame_enable_motor << 24 | self.master << 8 | self.motor #2進数のまま結合
#         bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
#         #print(hex(bin_num))
#         hex_str = hex(bin_num)[2:]
#         hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
#         print(hex_can)

#         self.ser.write(bytes.fromhex(hex_can))
#         self.ser.flush()

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         self.get_motor_state(received_data)


#     def stop_motor(self):

#         bin_num = frame_stop_motor << 24 | self.master << 8 | self.motor #2進数のまま結合
#         bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
#         #print(hex(bin_num))
#         hex_str = hex(bin_num)[2:]
#         hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
#         print(hex_can)

#         self.ser.write(bytes.fromhex(hex_can))
#         self.ser.flush()

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         self.get_motor_state(received_data)


#     def position_control(self, target_angle, speed):

#         bin_num = frame_write_param << 24 | self.master << 8 | self.motor #2進数のまま結合
#         bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
#         #print(hex(bin_num))
#         hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr

#         hex_angle_param = struct.unpack('!I', struct.pack('!f', target_angle))[0]
#         hex_angle_param = reverse_hex(format(hex_angle_param, "08x"))
#         #print(hex_angle_param)

#         hex_speed_param = struct.unpack('!I', struct.pack('!f', speed))[0]
#         hex_speed_param = reverse_hex(format(hex_speed_param, "08x"))
#         #print(hex_speed_param)

#         target_angle_hex = frame_head + hex_str + "08" + index_angle + "0000" + hex_angle_param + frame_tail
#         target_speed_hex = frame_head + hex_str + "08" + index_speed + "0000" + hex_speed_param + frame_tail

#         self.ser.write(bytes.fromhex(target_speed_hex))
#         self.ser.flush()
#         print(target_speed_hex)

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         self.get_motor_state(received_data)

#         self.ser.write(bytes.fromhex(target_angle_hex))
#         self.ser.flush()
#         print(target_angle_hex)

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         self.get_motor_state(received_data)


#     def homing_mode(self):

#         bin_num = frame_homing_mode << 24 | self.master << 8 | self.motor #2進数のまま結合
#         bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
#         #print(hex(bin_num))
#         hex_str = hex(bin_num)[2:]
#         hex_can = frame_head + hex_str + "080100000000000000" + frame_tail
#         print(hex_can)

#         self.ser.write(bytes.fromhex(hex_can))
#         self.ser.flush()

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         self.get_motor_state(received_data)

    
#     def get_motor_state(self, received_data):

#         status = int(received_data[4:12], 16) >> 3

#         frame_type = status >> 24

#         if(frame_type != 2):
#             print("Received data Error")
#             return 1

#         motor_can_id = status >> 8
#         motor_can_id = bin(motor_can_id)[-8:]
#         motor_can_id = int(motor_can_id, 2)

#         under_voltage = status >> 16
#         under_voltage = bin(under_voltage)[-1:]
#         under_voltage = int(under_voltage, 2)

#         over_current = status >> 17
#         over_current = bin(over_current)[-1:]
#         over_current = int(over_current, 2)

#         over_temp = status >> 18
#         over_temp = bin(over_temp)[-1:]
#         over_temp = int(over_temp, 2)

#         magnetic_encoding_failure = status >> 19
#         magnetic_encoding_failure = bin(magnetic_encoding_failure)[-1:]
#         magnetic_encoding_failure = int(magnetic_encoding_failure, 2)

#         HALL_encoding_failure = status >> 20
#         HALL_encoding_failure = bin(HALL_encoding_failure)[-1:]
#         HALL_encoding_failure = int(HALL_encoding_failure, 2)

#         not_calibrated = status >> 21
#         not_calibrated = bin(not_calibrated)[-1:]
#         not_calibrated = int(not_calibrated, 2)

#         print("Motor ID: ", motor_can_id)

#         if(under_voltage == 1):
#             print("Under Voltage Fault")
#         if(over_current == 1):
#             print("Over Current")
#         if(over_temp == 1):
#             print("Over Temperature")
#         if(magnetic_encoding_failure == 1):
#             print("Magnetic Encoding Failure")
#         if(HALL_encoding_failure == 1):
#             print("HALL Encoding Failure")
#         if(not_calibrated == 1):
#             print("Not Calibrated")

#         current_angle = int(received_data[14:18], 16)
#         current_angle_vel = int(received_data[18:22], 16)
#         current_torque = int(received_data[22:26], 16)
#         current_temp = int(received_data[26:30], 16)

#         angle = linear_mapping(current_angle, -4*math.pi, 4*math.pi)
#         angle_vel = linear_mapping(current_angle_vel, -30.0, 30.0)
#         torque = linear_mapping(current_torque, -12.0, 12.0)
#         temp = current_temp / 10.0

#         print("Angle:", angle, "rad")
#         print("Angle Velocity:", angle_vel, "rad/s")
#         print("Torque:", torque, "Nm")
#         print("Temperature:", temp, "℃")

#         self.angle = angle


#     def set_run_mode(self, mode):

#         bin_num = frame_write_param << 24 | self.master << 8 | self.motor #2進数のまま結合
#         bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
#         #print(hex(bin_num))
#         hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr

#         for key in run_mode:
#             if(key == mode):
#                 mode_data = run_mode[key]
#                 break
        
#         hex_can = frame_head + hex_str + "0805700000" + "0" + str(mode_data) + "000000" + frame_tail
#         print(hex_can)

#         self.ser.write(bytes.fromhex(hex_can))
#         self.ser.flush()

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         self.get_motor_state(received_data)


#     def power_on(self):

#         self.ser.write(bytes.fromhex('41 54 2b 41 54 0d 0a'))
#         self.ser.flush()
#         print('41542b41540d0a')

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         bin_num = self.master << 8 | self.motor #2進数のまま結合
#         bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
#         #print(hex(bin_num))
#         hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
#         hex_can = frame_head + "000" + hex_str + "0100" + frame_tail
#         print(hex_can)

#         self.ser.write(bytes.fromhex(hex_can))
#         self.ser.flush()

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         bin_num = frame_power_on << 24 | self.master << 8 | self.motor #2進数のまま結合
#         bin_num = bin_num << 3 | 0b100 #3bit左にずらし、右端を100にする
#         #print(hex(bin_num))
#         hex_str = hex(bin_num)[2:] #真ん中のデータ、16進数のstr
#         hex_can = frame_head + hex_str + "084066313130333103" + frame_tail
#         print(hex_can)

#         self.ser.write(bytes.fromhex(hex_can))
#         self.ser.flush()

#         data = self.ser.read_until(expected=b'\r\n')
#         data = int.from_bytes(data, "little")
#         received_data = reverse_hex("0"+hex(data)[2:])
#         print(">>" + received_data)

#         # self.get_motor_state(received_data)






class SubControllerNode(Node):
    def __init__(self):
        super().__init__('sub_controller_node')

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
    sub_controller_node = SubControllerNode()
    rclpy.spin(sub_controller_node)
    sub_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


