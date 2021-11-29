#!/usr/bin/env python3
# mqtt.py

import paho.mqtt.client as mqtt
import json
import enum
import asyncio


class myMqtt():
    def __init__(self):
        # prediction
        self.asl = str()
        self.gesture = str()

        self.id = '' # TODO: edit address
        self.GESTURE_RESPONSE_TOPIC = 'group7/gesture/prediction'
        self.ASL_RESPONSE_TOPIC = 'group7/asl/prediction'
        self.GESTURE_REQUEST_TOPIC = 'group7/gesture/request'
        self.ASL_REQUEST_TOPIC = 'group7/asl/request'
        self.exit = False
        
        self.client = mqtt.Client()

        # callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    async def run(self):
        # connect
        self.client.connect(self.id)
        self.client.loop_start()

        # blocking loop
        while not self.exit:
            await asyncio.sleep(0.01)


    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f'==========   Connected to {self.id}   ==========')
            client.subscribe(self.GESTURE_RESPONSE_TOPIC)
            client.subscribe(self.ASL_RESPONSE_TOPIC)
        else:
            print(f'Failed to connect. Error code: {rc}.')
        
    def on_message(self, client, userdata, msg):
        '''
        prediction structure:
        {
            'mode': 'ASL' or 'GESTURE',
            'response': prediction
        }
        '''
        # for ASL
        if msg.topic == self.ASL_RESPONSE_TOPIC:
            resp_dict = json.loads(msg.payload)
            # updates prediction
            self.asl = resp_dict['response']

        # for Gesture
        elif msg.topic == self.GESTURE_RESPONSE_TOPIC:
            resp_dict = json.loads(msg.payload)
            # updates prediction
            self.gesture = resp_dict['response']

        else:
            # resets prediction
            self.gesture = str()
            self.asl = str()

        # for debugging purposes
        print(f'prediction = {self.gesture if self.gesture != "IDLE" else self.asl}, isIdle = {"True" if self.gesture == "IDLE" else "False"}')


    def publish(self, mode, raw_data):
        '''
        packet structure:
        {
            'mode': 'ASL' or 'GESTURE',
            'raw': raw_data
        }
        '''
        payload = json.dumps(
            {
                'mode': mode,
                'raw': raw_data
            }
        )
        self.client.publish(self.GESTURE_REQUEST_TOPIC if mode == MODE.GESTURE.name else self.ASL_REQUEST_TOPIC, payload)
    
    def disconnect(self):
        self.client.disconnect()
        self.exit = True
        print(f'==========   Disconnected to {self.id}   ==========')


class MODE(enum.Enum):
    GESTURE = 1
    ASL = 2

