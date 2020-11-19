#!/usr/bin/python3
# coding: utf-8
# encoding=utf-8

from machine import I2C
import utime
import time
import binascii
import ustruct
from fpioa_manager import *
from Maix import GPIO
#import threading
#import _thread
import sensor
import image
import lcd
import KPU as kpu


class Maix_dock_device:

    VL53L0X_addr    =   0x29
    memaddr         =   0xc2
    nbytes          =   1


    VL53L0X_REG_IDENTIFICATION_MODEL_ID     =   0xc0
    VL53L0X_REG_IDENTIFICATION_REVISION_ID  =   0xc2
    VL53L0X_REG_RESULT_INTERRUPT_STATUS     =   0x13
    VL53L0X_REG_RESULT_RANGE_STATUS         =   0x14
    VL53L0X_REG_SYSRANGE_START              =   0x00
    pcf8591_addr    =   0x48


    fm.register( 1,fm.fpioa.GPIO0)
    fm.register( 2,fm.fpioa.GPIO1)
    fm.register( 3,fm.fpioa.GPIO2)
    fm.register( 13,fm.fpioa.GPIO3)
    fm.register( 14,fm.fpioa.GPIO4)
    fm.register( 15,fm.fpioa.GPIOHS1)
    fm.register( 17,fm.fpioa.GPIOHS2)

    UVC         = GPIO(GPIO.GPIO0,GPIO.OUT)
    LED_dis     = GPIO(GPIO.GPIO1,GPIO.OUT)
    BEE         = GPIO(GPIO.GPIO2,GPIO.OUT)
    TEMP_1      = GPIO(GPIO.GPIO3,GPIO.OUT)
    TEMP_2      = GPIO(GPIO.GPIO4,GPIO.OUT)
    KEY_start   = GPIO(GPIO.GPIOHS1,GPIO.IN,GPIO.IRQ_FALLING)
    #key = GPIO(GPIO.GPIOHS0, GPIO.IN, GPIO.PULL_NONE)
    KEY_lock    = GPIO(GPIO.GPIOHS2,GPIO.IN,GPIO.PULL_NONE)


    tempBorder_times = 0


    lcd.init()
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.run(1)
    task = kpu.load(0x300000) # you need put model(face.kfpkg) in flash at address 0x300000
    # task = kpu.load("/sd/face.kmodel")
    anchor = (1.889, 2.5245, 2.9465, 3.94056, 3.99987, 5.3658, 5.155437, 6.92275, 6.718375, 9.01025)
    a = kpu.init_yolo2(task, 0.5, 0.3, 5, anchor)


    i2c = I2C(I2C.I2C0, mode=I2C.MODE_MASTER, freq=100000, scl=9, sda=10) # software i2c
    i2c_extend = I2C(I2C.I2C1, mode=I2C.MODE_MASTER, freq=100000, scl=11, sda=12) # software i2c


    def __init__(self):
        print("Maix_dock_device class")
        self.infrared_range_left        = 0
        self.infrared_range_right       = 0
        self.temperature_sensor_left    = 0
        self.temperature_sensor_right   = 0
        self.ad48v                      = 0
        self.face_detection             = 0


    def set_face_detection(self):
        img = sensor.snapshot()
        code = kpu.run_yolo2(self.task, img)
        if code:
            for i in code:
                #print(i)
                #a = img.draw_rectangle(i.rect())
                print("find face")
                self.face_detection =   1
        else:
            print("no face")
            self.face_detection =   0


    def get_face_detection(self):
        return self.face_detection


    def get_infrared_range_left(self):
        return self.infrared_range_left


    def get_infrared_range_right(self):
        return self.infrared_range_right


    def get_ad48v(self):
        return self.ad48v


    def get_temperature_sensor_left(self):
        return self.temperature_sensor_left


    def get_temperature_sensor_left(self):
        return self.temperature_sensor_left


    def set_UVC_OUT(self,value):
        self.UVC.value(value)


    def set_led(self,value):
        self.LED_dis.value(value)


    def get_temperature_sensor_left(self):
        return 20.1


    def get_keyValue_start(self):
        if(self.KEY_start.value() == 1):
            time.sleep_ms(10)
            if(self.KEY_start.value() == 1):
                #print("KEY_start close")
                return 1
        else :
            #print("KEY_start open")
            return 0


    def show_deviceAddr(self):
        devices = self.i2c.scan()
        print(devices)
        devices2 = self.i2c_extend.scan()
        print(devices2)


    def set_infrared_range_left(self):
        try:
            self.i2c.writeto_mem(0x29,0x00,chr(1))
            time.sleep_ms(10)
            data_i2c = self.i2c.readfrom_mem(0x29,0x1e,2)
            data_i2c = data_i2c[0]<< 8 | data_i2c[1]
            if data_i2c != 20:
                self.infrared_range_left = data_i2c
                return data_i2c
            else :
                print("data_i2c == 20")
                return None
        except OSError as err:
            if err.args[0] == errno.EIO:
                print("i2c1 dis errno.EIO")
                return None
        else:
            print("i2c1 abnormal")
            return None

    def set_infrared_range_right(self):
        try:
            self.i2c_extend.writeto_mem(0x29,0x00,chr(1))
            #self.i2c_extend.writeto_mem(0x21,0x00,chr(1))
            time.sleep_ms(10)
            data_i2c = self.i2c_extend.readfrom_mem(0x29,0x1e,2)
            #data_i2c = self.i2c_extend.readfrom_mem(0x21,0x1e,2)
            data_i2c = data_i2c[0]<< 8 | data_i2c[1]
            if data_i2c != 20:
                self.infrared_range_right = data_i2c
                return data_i2c
            else :
                print("data_i2c == 20")
                return None
        except OSError as err:
            if err.args[0] == errno.EIO:
                print("i2c2 errno.EIO")
                return None
        else:
            print("i2c2 abnormal")
            return None

    def set_ad48v_chl(self,chn):
        try:
            if chn == 0:
                self.i2c.writeto(self.pcf8591_addr, chr(0x40))
            if chn == 1:
                self.i2c.writeto(self.pcf8591_addr, chr(0x41))
            if chn == 2:
                self.i2c.writeto(self.pcf8591_addr, chr(0x42))
            if chn == 3:
                self.i2c.writeto(self.pcf8591_addr, chr(0x43))
            self.i2c.readfrom(self.pcf8591_addr, 1)
            ad_value    =   self.i2c.readfrom(self.pcf8591_addr, 1)
            ad_value    =   ad_value[0]*48/255
            self.ad48v   =   ad_value
            return ad_value
        except OSError as err:
            if err.args[0] == errno.EIO:
                print("i2c1 ad errno.EIO")


    def set_ad48v(self):
        self.set_ad48v_chl(0)


    def uvc_autoControl(self):
        tempBorder_times = 0
        self.set_infrared_range_left()
        utime.sleep_ms(200)
        self.set_ad48v()
        utime.sleep_ms(200)
        self.set_infrared_range_right()
        utime.sleep_ms(200)
        self.set_face_detection()
        if self.get_keyValue_start():
            if((self.get_face_detection() == 0) and (self.get_ad48v() > 36)):
                if((tempBorder_times<=3) and ((self.get_infrared_range_left()<500) or (self.get_infrared_range_right()<500))):
                    self.set_UVC_OUT(1);
                    self.set_led(1);
                    print("uvc_out: 1");
                else:
                    self.set_UVC_OUT(0);
                    self.set_led(0);
                    print("uvc_out: 0");
            print("key_start_down!");
        else:
            print("key_start_up!");
            self.set_UVC_OUT(0);
            self.set_led(0);

        print("infrared_range_left  = ",self.get_infrared_range_left())
        print("infrared_range_right = ",self.get_infrared_range_right())
        print("ad48v = ",self.get_ad48v())
        print("face_detection = ",self.get_face_detection())

        #if(get_temperature_sensor_left()>70):
            #tempBorder_times += 1;
            #if(tempBorder_times>3):
                #tempBorder_times=4;
        #else:
            #tempBorder_times = 0


def main():
    myDevice = Maix_dock_device()
    myDevice.show_deviceAddr()
    while True :
        myDevice.uvc_autoControl()
        time.sleep_ms(500)

    #a = kpu.deinit(myDevice.task)

if __name__ == "__main__" :
    main()




