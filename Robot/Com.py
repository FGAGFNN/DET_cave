# -*- coding: utf-8 -*-
# CRC校验  确保数据的正确性
import struct
import serial
import time
import copy
CRC_Table = (
    0X00, 0X07, 0X0E, 0X09, 0X1C, 0X1B, 0X12, 0X15, 0X38, 0X3F, 0X36, 0X31, 0X24, 0X23, 0X2A, 0X2D,
    0X70, 0X77, 0X7E, 0X79, 0X6C, 0X6B, 0X62, 0X65, 0X48, 0X4F, 0X46, 0X41, 0X54, 0X53, 0X5A, 0X5D,
    0XE0, 0XE7, 0XEE, 0XE9, 0XFC, 0XFB, 0XF2, 0XF5, 0XD8, 0XDF, 0XD6, 0XD1, 0XC4, 0XC3, 0XCA, 0XCD,
    0X90, 0X97, 0X9E, 0X99, 0X8C, 0X8B, 0X82, 0X85, 0XA8, 0XAF, 0XA6, 0XA1, 0XB4, 0XB3, 0XBA, 0XBD,
    0XC7, 0XC0, 0XC9, 0XCE, 0XDB, 0XDC, 0XD5, 0XD2, 0XFF, 0XF8, 0XF1, 0XF6, 0XE3, 0XE4, 0XED, 0XEA,
    0XB7, 0XB0, 0XB9, 0XBE, 0XAB, 0XAC, 0XA5, 0XA2, 0X8F, 0X88, 0X81, 0X86, 0X93, 0X94, 0X9D, 0X9A,
    0X27, 0X20, 0X29, 0X2E, 0X3B, 0X3C, 0X35, 0X32, 0X1F, 0X18, 0X11, 0X16, 0X03, 0X04, 0X0D, 0X0A,
    0X57, 0X50, 0X59, 0X5E, 0X4B, 0X4C, 0X45, 0X42, 0X6F, 0X68, 0X61, 0X66, 0X73, 0X74, 0X7D, 0X7A,
    0X89, 0X8E, 0X87, 0X80, 0X95, 0X92, 0X9B, 0X9C, 0XB1, 0XB6, 0XBF, 0XB8, 0XAD, 0XAA, 0XA3, 0XA4,
    0XF9, 0XFE, 0XF7, 0XF0, 0XE5, 0XE2, 0XEB, 0XEC, 0XC1, 0XC6, 0XCF, 0XC8, 0XDD, 0XDA, 0XD3, 0XD4,
    0X69, 0X6E, 0X67, 0X60, 0X75, 0X72, 0X7B, 0X7C, 0X51, 0X56, 0X5F, 0X58, 0X4D, 0X4A, 0X43, 0X44,
    0X19, 0X1E, 0X17, 0X10, 0X05, 0X02, 0X0B, 0X0C, 0X21, 0X26, 0X2F, 0X28, 0X3D, 0X3A, 0X33, 0X34,
    0X4E, 0X49, 0X40, 0X47, 0X52, 0X55, 0X5C, 0X5B, 0X76, 0X71, 0X78, 0X7F, 0X6A, 0X6D, 0X64, 0X63,
    0X3E, 0X39, 0X30, 0X37, 0X22, 0X25, 0X2C, 0X2B, 0X06, 0X01, 0X08, 0X0F, 0X1A, 0X1D, 0X14, 0X13,
    0XAE, 0XA9, 0XA0, 0XA7, 0XB2, 0XB5, 0XBC, 0XBB, 0X96, 0X91, 0X98, 0X9F, 0X8A, 0X8D, 0X84, 0X83,
    0XDE, 0XD9, 0XD0, 0XD7, 0XC2, 0XC5, 0XCC, 0XCB, 0XE6, 0XE1, 0XE8, 0XEF, 0XFA, 0XFD, 0XF4, 0XF3
)
class Serial(object):
    def __init__(self,com_id='COM4',rate=19200):
        super(Serial, self).__init__()
        self.id = com_id
        self.rate = rate
        self.com = serial.Serial(self.id, self.rate, timeout=0.05)
        
    def CRC_Dector(self,data,data_lenth):
        crc8 = 0
        cnt = 0
        while data_lenth > 0:
            data_lenth = data_lenth - 1
            crc8 = crc8 ^ (data[cnt])
            crc8 = CRC_Table[crc8]
            cnt = cnt + 1
        # print("crc8:%x"%crc8)
        return crc8

            
    def push_other_inform(self,order): #发送没有位置
        byte = struct.pack("<3B1H1B3h", 171, 186, order,12, 0, 0, 0, 0)
        # AB BA 01 length(两个字节) 00  x y z CRC
        crc = self.CRC_Dector(byte, 12)
        # print("CRC:%x"%crc)
        pack_byte = struct.pack("<3B1H1B3h1B", 171, 186, order,12, 0, 0, 0, 0, crc)
        # print("发送没有位置:" )
        # for i in pack_byte:
        #     print("%02x"%i,end=' ')
        self.com.write(pack_byte)
        #self.com.flushInput()
        self.com.flushOutput()
    def push_right_axis2(self,out,order): #发送坐标
        # AB BA 01 length（两个字节） 目标数量 x(两个字节) y(两个字节) z(两个字节) angle(两个字节) .... crc
        
        print(out) #[x y z]
        number = len(out)//3 #目标数量
        if number>=20:#最多不超过20个目标
            out = out[:60]
            number = 20
        length = number*(3*2)+ 6 #数据长度
        _struck ='3B1H1B'
        for _ in range(number):
            _struck +='3h'
        byte = struct.pack('<{}'.format(_struck), 171, 186, order,length, number,*out)
        crc = self.CRC_Dector(byte, 6+6*number)
        out.append(crc)
        _struck +='1B'
        pack_byte = struct.pack('<{}'.format(_struck), 171, 186, order, length,number,*out)
        self.com.write(pack_byte)
        #self.com.flushInput()
        self.com.flushOutput()

        # byte = struct.pack("<4B4h1B", 171, 186, 14, 1, *out, 0, 0)
        # # AB BA 01 length(两个字节) 00 CRC
        # crc = self.CRC_Dector(byte, 13)
        # pack_byte = struct.pack("<4B4h2B", 171, 186, 14, 1, *out, 0, 0, crc)
        # self.com.write(pack_byte)
        # #self.com.flushInput()
        # self.com.flushOutput()
