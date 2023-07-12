#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import struct
import time
import threading
import serial as s
from serial_pkgs.msg import Cmd, MotorState
from std_msgs.msg import Int16


class serial_module:
    def __init__(self):
        # initidata
        # self.port = "/dev/ttyUSB0"  # port
        # self.port = ['/dev/ttyUSB0', '/dev/ttyUSB1',
        #              '/dev/ttyUSB2', "/dev/ttyUSB3"]
        self.port = ['/dev/bus/usb/003/005']
        self.ser = None
        self.baudrate = 1000000
        # command list : get the float num inside the "[]" of each command
        self.receve_command: dict = {"P": 0, "N": 4, "S": 4}
        self.commands = ["S", "N", "P"]
        # subseribe and advertise the topic of pubish serial
        self.motor_state = MotorState()
        rospy.Subscriber("/serial/pub", Cmd,
                         self.serial_pub_cb, queue_size=100)
        # motor state from serial
        self.MotorState_Pub = rospy.Publisher(
            "/serial/MotorState", MotorState, queue_size=5)
        # reflaction after the Position Control
        self.PosControl_Pub = rospy.Publisher(
            "/serial/PosControl", Int16, queue_size=1)

        # connect the serial
        num = 0
        get_msg = False
        while not get_msg:
            # waiting while to successful connnect
            try:
                rospy.logwarn("waiting for the serial:"+self.port[num])
                self.ser = s.Serial(port=self.port[num], baudrate=self.baudrate, bytesize=8,
                                    stopbits=1, timeout=0.2, rtscts=True, dsrdtr=True)
            except:
                pass
            if self.ser is None:
                rospy.sleep(0.2)
                continue
            num = (num+1) % len(self.port)
            rospy.sleep(1)
            get_byte = self.ser.in_waiting
            # 判断
            if get_byte >= 10:  # 正常反馈电机数据为 8float + 3 = 35 byte
                get_msg = True
            else:
                rospy.logerr(
                    "get refalct unnormally,switch port...."
                )

        rospy.loginfo("Successfully connect the serial")

        # create a new thread to continuly
        threading.Thread(target=self.serial_read).start()

        rospy.spin()

    # read serial data and process
    def serial_read(self):
        while True:
            command = self.ser.read(1).decode("utf-8", errors='ignore')
            # print("[", rospy.Time.now(), "]", "get serial data:", command)
            if command not in self.commands:
                continue
            start = self.ser.read(1).decode("utf-8", errors="ignore")
            # print("[", rospy.Time.now(), "]", "get serial data:", start)
            if start == "[":
                # command = command[0]
                # if exist,return the float num of current command else none
                float_num = self.receve_command.get(command, None)
                if float_num == None:
                    rospy.logerr("Undefined Command Named[%s]".format(command))
                    # read till the end of one command
                    # continue
                    while True:
                        command = self.ser.read(1).decode(
                            "utf-8", errors='ignore')
                        # print("[", rospy.Time.now(), "]",
                        #       "get serial data:", command)
                        if command == ']':
                            break
                    continue
                if float_num > 0:  # have reflect data
                    unpack_data = self.ser.read(
                        4*float_num)  # 4 byte per float
                    datas = struct.unpack(
                        str(float_num)+"f", unpack_data)  # str(float_num)+
                    # print("[", rospy.Time.now(), "]",
                    #       "get serial data:", command, ":", datas)
                # process data and publish to the topic net
                    # MotorState
                    if command == "S":
                        self.motor_state.Speed = datas
                        self.MotorState_Pub.publish(self.motor_state)
                    # other command
                    elif command == "N":
                        self.motor_state.Range = datas
                        self.MotorState_Pub.publish(self.motor_state)
                # no data relfect
                # position control reflect
                elif float_num == 0:
                    if command == "P":
                        topic = Int16()
                        topic.data = 0
                        self.PosControl_Pub.publish(topic)
                        print("[", rospy.Time.now(), "]",
                          "get serial data: finish position control.")
                    # other command
                    elif command == "X":
                        pass
                # get the end of command
                command = self.ser.read(1).decode("utf-8", errors='ignore')
                # print("[", rospy.Time.now(), "]", "get serial data:", command)
                if command == "]":
                    continue
                else:
                    rospy.logerr(
                        "Unexpected End of the command[%s]".format(command))
                    continue
            else:
                rospy.logerr(
                    "Unexpected Command format[%s....]".format(command))
                # self.ser.read_all()  # clear the serial buffer
                # read till the end of one command
                while True:
                    command = self.ser.read(1).decode("utf-8", errors='ignore')
                    # print("[", rospy.Time.now(), "]",
                        #   "get serial data:", command)
                    if command == ']':
                        break
                continue

    # publish the topic CMD to serial
    def serial_pub_cb(self, msg: Cmd):
        if len(msg.cmd) != 1:
            rospy.logerr("msg [" + msg.cmd + "] doesn`t match the length of 1")
            return
        # Publish "Reset" to the stm32
        if msg.cmd == "C":
            for i in range(5):
                rospy.logwarn("reseting stm32 now")
                self.ser.write(bytes([0xff]))

        else:
            print("[", rospy.Time.now(), "]",
                  "publishing:", msg.cmd, msg.value)
            # publish the serial signal
            self.ser.write((msg.cmd + "[").encode("utf-8"))
            for i in msg.value:
                self.ser.write(struct.pack('f', i))
            self.ser.write("]".encode("utf-8"))


if __name__ == "__main__":
    rospy.init_node("Serial_Node", anonymous=True)
    modu = serial_module()
