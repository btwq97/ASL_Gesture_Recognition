#!/usr/bin/env python3 -W ignore::DeprecationWarning
# gesture.py

import struct
import asyncio

from bleak import BleakClient
from mqtt import MODE


'''
Globals
'''
gx = 0.0
gy = 0.0
gz = 0.0
ax = 0.0
ay = 0.0
az = 0.0


class Service:
    """
    Here is a good documentation about the concepts in ble;
    https://learn.adafruit.com/introduction-to-bluetooth-low-energy/gatt

    In TI SensorTag there is a control characteristic and a data characteristic which define a service or sensor
    like the Light Sensor, Humidity Sensor etc

    Please take a look at the official TI user guide as well at
    https://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide
    """
    def __init__(self):
        self.data_uuid = None
        self.ctrl_uuid = None

class Sensor(Service):
    def callback(self, sender: int, data: bytearray):
        raise NotImplementedError()

    async def start_listener(self, client, *args):
        # start the sensor on the device
        write_value = bytearray([0x01])
        await client.write_gatt_char(self.ctrl_uuid, write_value)

        # listen using the handler
        await client.start_notify(self.data_uuid, self.callback)

class MovementSensorMPU9250SubService:
    def __init__(self):
        self.bits = 0

    def enable_bits(self):
        return self.bits

    def cb_sensor(self, data):
        raise NotImplementedError

class MovementSensorMPU9250(Sensor):
    GYRO_XYZ = 7
    ACCEL_XYZ = 7 << 3
    MAG_XYZ = 1 << 6
    ACCEL_RANGE_2G  = 0 << 8
    ACCEL_RANGE_4G  = 1 << 8
    ACCEL_RANGE_8G  = 2 << 8
    ACCEL_RANGE_16G = 3 << 8

    def __init__(self):
        super().__init__()
        self.data_uuid = "f000aa81-0451-4000-b000-000000000000"
        self.ctrl_uuid = "f000aa82-0451-4000-b000-000000000000"
        self.ctrlBits = 0

        self.sub_callbacks = []

    def register(self, cls_obj: MovementSensorMPU9250SubService):
        self.ctrlBits |= cls_obj.enable_bits()
        self.sub_callbacks.append(cls_obj.cb_sensor)

    async def start_listener(self, client, *args):
        # start the sensor on the device
        await client.write_gatt_char(self.ctrl_uuid, struct.pack("<H", self.ctrlBits))

        # listen using the handler
        await client.start_notify(self.data_uuid, self.callback)

    def callback(self, sender: int, data: bytearray):
        unpacked_data = struct.unpack("<hhhhhhhhh", data)
        for cb in self.sub_callbacks:
            cb(unpacked_data)

class AccelerometerSensorMovementSensorMPU9250(MovementSensorMPU9250SubService):
    def __init__(self):
        self.bits = MovementSensorMPU9250.ACCEL_XYZ | MovementSensorMPU9250.ACCEL_RANGE_4G
        self.scale = 8.0/32768.0 # TODO: why not 4.0, as documented? @Ashwin Need to verify

    def cb_sensor(self, data):
        global ax, ay, az
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        ax = data[3]*self.scale
        ay = data[4]*self.scale
        az = data[5]*self.scale


class GyroscopeSensorMovementSensorMPU9250(MovementSensorMPU9250SubService):
    def __init__(self):
        self.bits = MovementSensorMPU9250.GYRO_XYZ
        self.scale = 500.0/65536.0

    def cb_sensor(self, data):
        global gx, gy, gz
        '''Returns (x_gyro, y_gyro, z_gyro) in units of degrees/sec'''
        gx = data[0]*self.scale
        gy = data[1]*self.scale
        gz = data[2]*self.scale
        

class gesture_recognition():
    def __init__(self, mqClient):
        self.client = mqClient
        self.uuid = '54:6C:0E:B7:B7:04' # TODO: edit SensorTag UUID
        self.exit = False
        self.data = list()

    def disconnect(self):
        self.exit = True
        # program exits
        print(f'==========   SensorTag {self.uuid} is disconnected   ==========')

    def append_data(self):
        global ax, ay, az, gx, gy, gz
        one_interval = (ax, ay, az, gx, gy, gz)
        self.data.append(one_interval)

    async def run(self):
        count = 0
        async with BleakClient(self.uuid) as client:
            # waits until SensorTag is connected
            status = await client.is_connected()
            print(f'==========   SensorTag {self.uuid} is connected   ==========')

            # enable gyro and acc
            acc = AccelerometerSensorMovementSensorMPU9250()
            gyro = GyroscopeSensorMovementSensorMPU9250()

            # init mpu sensor
            mpu9250 = MovementSensorMPU9250()
            mpu9250.register(acc)
            mpu9250.register(gyro)

            # listens to sensor for inputs
            await mpu9250.start_listener(client)

            # blocking loop
            while not self.exit:
                await asyncio.sleep(0.1) 

                self.append_data()
                count += 1

                '''
                Publish to cloud for prediction
                '''
                if count == 10:
                    self.client.publish(MODE.GESTURE.name, self.data)
                    self.data.clear()
                    count = 0


