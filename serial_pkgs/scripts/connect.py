#!/usr/bin/env python3
import rospy
import struct
import threading
import serial as s
from serial_pkgs.msg import Cmd, MotorState
from std_msgs.msg import Int16

flag_pack2ascii = 1234567.0


class serial_module:
    def __init__(self):
        # initidata
        # self.port = "/dev/ttyUSB0"  # port
        # self.port = ['/dev/ttyUSB0', '/dev/ttyUSB1',
        #              '/dev/ttyUSB2', "/dev/ttyUSB3"]

        # 指定USB口固定
        self.port_up = '/dev/ser_up'  # 顶层串口
        self.port_down = '/dev/ser_down'  # 底层串口
        # 串口变量
        self.ser_up = None
        self.ser_down = None
        # 波特率
        self.baudrate = 1000000
        # 链接串口
        while self.ser_down is None:
            try:
                rospy.logwarn("等待下层")
                self.ser_down = s.Serial(port=self.port_down, baudrate=self.baudrate,
                                         bytesize=8, stopbits=1, timeout=1, rtscts=True, dsrdtr=True)
                self.ser_down.read(1)
            except Exception as e:
                rospy.logerr("在链接串口的过程中抛出异常:"+str(e))
        rospy.loginfo("成功连接下层串口")
        while self.ser_up is None:
            try:
                rospy.logwarn("等待上层")
                self.ser_up = s.Serial(port=self.port_up, baudrate=115200,
                                       bytesize=8, stopbits=1, timeout=0.2, rtscts=True, dsrdtr=True)
            except Exception as e:
                rospy.logerr("在链接串口的过程中抛出异常:"+str(e))
        rospy.loginfo("成功连接上层串口")

        self.receve_command = {"P": 0, "N": 4, "S": 4}  # 字典数据数量
        self.commands = ["S", "N", "P"]  # 命令集
        # subseribe and advertise the topic of pubish serial
        self.motor_state = MotorState()
        rospy.Subscriber("/serial/pub", Cmd,
                         self.serial_pub_cb, queue_size=100, callback_args=(self.ser_down))  # 下层通信
        rospy.Subscriber("/serial/pub_up", Cmd,
                         self.serial_pub_cb, queue_size=100, callback_args=(self.ser_up))  # 上层通信
        # motor state from serial
        self.MotorState_Pub = rospy.Publisher(
            "/serial/MotorState", MotorState, queue_size=5)
        # reflaction after the Position Control
        self.PosControl_Pub = rospy.Publisher(
            "/serial/PosControl", Int16, queue_size=1)

        # # connect the serial
        # num = 0
        # get_msg = False
        # while not get_msg:
        #     # waiting while to successful connnect
        #     try:
        #         rospy.logwarn("waiting for the serial:"+self.port[num])
        #         self.ser = s.Serial(port=self.port[num], baudrate=self.baudrate, bytesize=8,
        #                             stopbits=1, timeout=0.2, rtscts=True, dsrdtr=True)
        #     except:
        #         pass
        #     if self.ser is None:
        #         rospy.sleep(0.2)
        #         continue
        #     num = (num+1) % 3
        #     rospy.sleep(0.5)
        #     get_byte = self.ser.in_waiting
        #     # 判断
        #     if get_byte >= 20:  # 正常反馈电机数据为 8float + 3 = 35 byte
        #         get_msg = True
        #     else:
        #         rospy.logerr(
        #             "get refalct unnormally,switch port...."
        #         )

        # 分别新建上下层的串口通信
        threading.Thread(target=self.serial_read,
                         args=(self.ser_down,)).start()
        threading.Thread(target=self.serial_read,
                         args=(self.ser_up,)).start()

        rospy.spin()

    # 读取串口数据和处理的函数
    def serial_read(self, serial):  # 链接上的串口变量,命令字符,命令的数据量(浮点个数)
        if serial == self.ser_up:
            ser_name = "上层"
        elif serial == self.ser_down:
            ser_name = "底盘"
        else:
            ser_name = "Err"
        while True:
            # 读取第一个字节,同时为命令字符
            command = serial.read(1).decode("utf-8", errors='ignore')
            if command not in self.commands:
                continue
            start = serial.read(1).decode("utf-8", errors="ignore")
            if start == "[":
                # 如果存在该命令,返回对应的浮点字符数
                float_num = self.receve_command.get(command, None)
                if float_num == None:
                    rospy.logerr(
                        "[%s] 未被定义的命令名称[%s]".format(ser_name, command))
                    # 错误 持续读到命令尾截止 "]"
                    # continue
                    while True:
                        command = serial.read(1).decode(
                            "utf-8", errors='ignore')
                        print("[", rospy.Time.now(), "]",
                              "读取串口数据:", command)
                        if command == ']':
                            break
                    continue
                if float_num > 0:  # 命令含有浮点数
                    unpack_data = serial.read(
                        4*float_num)  # 4 每个浮点数4字节
                    datas = struct.unpack(
                        str(float_num)+"f", unpack_data)  # str(float_num)+
                    # print("[", rospy.Time.now(), "][", ser_name, "]",
                    #       "读取串口数据:", command, ":", datas)
                # 处理数据并发布到ros话题网络
                    # 电机速度
                    if command == "S":
                        self.motor_state.Speed = datas
                        self.MotorState_Pub.publish(self.motor_state)
                    # 电机里程
                    elif command == "N":
                        self.motor_state.Range = datas
                        self.MotorState_Pub.publish(self.motor_state)
                # 没有浮点数的标志位(触发式)命令
                # 位控结束的返回帧
                elif float_num == 0:
                    if command == "P":
                        topic = Int16()
                        topic.data = 0
                        self.PosControl_Pub.publish(topic)
                    # 其他命令
                    elif command == "X":
                        pass
                #
                command = serial.read(1).decode("utf-8", errors='ignore')
                # print("[", rospy.Time.now(), "]", "get serial data:", command)
                if command == "]":
                    continue
                else:
                    rospy.logerr(
                        "[%s] 不合法的命令结尾[%s]".format(ser_name, command))
                    continue
            # 没有以"["开头包裹数据
            else:
                rospy.logerr(
                    "[%s] 不合法的命令开头[%s....]".format(ser_name, command))
                # 读取到命令尾部
                while True:
                    command = self.ser.read(1).decode("utf-8", errors='ignore')
                    # print("[", rospy.Time.now(), "]",
                    #       "get serial data:", command)
                    if command == ']':
                        break
                continue

    # 发布命令到串口
    def serial_pub_cb(self, msg: Cmd, serial):  # Cmd 命令话题,serial 指定串口(上层或下层)
        if serial == self.ser_up:
            ser_name = "上层"
        elif serial == self.ser_down:
            ser_name = "底盘"
        else:
            ser_name = "Err"

        if len(msg.cmd) != 1:
            rospy.logerr("["+ser_name+"] 命令[" + msg.cmd + "]长度不为1")
            return
        # Publish "Reset" to the stm32
        if msg.cmd == "C" and msg.value == []:
            rospy.logwarn("["+ser_name+"]软复位主控中...")
            for i in range(5):
                serial.write(bytes([0xff]))

        else:

            # 发布为串口信号
            # serial.write((msg.cmd + "[").encode("utf-8"))
            # 约定发送字符串的格式
            if len(msg.value) > 1 and int(msg.value[0]) == int(flag_pack2ascii):
                text = msg.cmd + '['
                # print("[", rospy.Time.now(), "][", ser_name, "]",
                #       "发布串口命令:", msg.cmd, "字符:", end='')
                for i in msg.value[1:]:
                    # print("\"", chr(int(i)), "\" ")
                    text += chr(int(i))
                    # serial.write(chr(int(i)).encode("utf-8"))  # 浮点数转整形再按ASCII转字符
                # print("")
                text += ']'
                serial.write(text.encode("utf-8"))  # 集合到一起减少帧与帧之间的间隔
            else:
                buf = (msg.cmd+'[').encode("utf-8")
                # print("[", rospy.Time.now(), "][", ser_name, "]",
                #      "发布串口命令:", msg.cmd, "数据:", msg.value)
                for i in msg.value:
                    buf += struct.pack('f', i)
                    # serial.write(struct.pack('f', i))
                buf += ']'.encode("utf-8")
                serial.write(buf)  # 集合到一起 减少帧与帧之间的间隔


if __name__ == "__main__":
    rospy.init_node("Serial_Node", anonymous=True)
    modu = serial_module()
