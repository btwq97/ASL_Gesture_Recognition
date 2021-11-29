#!/usr/bin/env python3
# mqtt.py

import paho.mqtt.client as mqtt
import json
import enum
import pickle
import tensorflow as tf
import numpy as np

from time import sleep
from sklearn.preprocessing import LabelEncoder


class myMqtt():
    def __init__(self):
        self.id = '172.28.227.52' # TODO: edit address
        self.GESTURE_RESPONSE_TOPIC = 'group7/gesture/prediction'
        self.ASL_RESPONSE_TOPIC = 'group7/asl/prediction'
        self.GESTURE_REQUEST_TOPIC = 'group7/gesture/request'
        self.ASL_REQUEST_TOPIC = 'group7/asl/request'
        self.exit = False
        self.useNN = True # TODO: NN or KNN
        self.pickleModel = './models/knn2.sav'
        self.hd5Model = './models/AZnn.hd5'

        self.GESTURE = ["IDLE", "HELLO", "PLEASE", "THANKS"]
        self.gesture_model = tf.keras.models.load_model('./models/GESTURE_MODEL')

        self.ALPHABETS = ['A', 'B', 'C', 'D', 'E', 'F',\
                        'G', 'H', 'I', 'J', 'K', 'L', \
                        'M', 'N', 'O', 'P', 'Q', 'R', \
                        'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z']
        self.asl_model = tf.keras.models.load_model(self.hd5Model) if self.useNN else pickle.load(open(self.pickleModel, 'rb'))
        self.label_encoder = LabelEncoder()

        self.client = mqtt.Client()

        # callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.label_encoder.fit_transform(self.ALPHABETS)

    def run(self):
        # connect
        self.client.connect(self.id)
        self.client.loop_start()

        # blocking loop
        while not self.exit:
            sleep(0.01)


    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print(f'==========   Connected to {self.id}   ==========')
            print(f'==========   ASL Model used = {self.hd5Model if self.useNN else self.pickleModel}   ==========')
            print(f'==========   Gesture Model used = ./models/GESTURE_MODEL   ==========')
            client.subscribe(self.GESTURE_REQUEST_TOPIC)
            client.subscribe(self.ASL_REQUEST_TOPIC)
        else:
            print(f'Failed to connect. Error code: {rc}.')
        
    def on_message(self, client, userdata, msg):
        '''
        packet structure:
        {
            'mode': 'ASL' or 'GESTURE',
            'raw': raw_data
        }
        '''
        prediction = str()
        resp_dict = dict()

        # for ASL
        if msg.topic == self.ASL_REQUEST_TOPIC:
            # request payload
            resp_dict = json.loads(msg.payload)

            temp = self.asl_model.predict(resp_dict['raw'])
            # prediction
            if self.useNN:
                index = np.argmax(temp)
                prediction = self.label_encoder.inverse_transform(np.array([index]))[0]
            else:
                prediction = self.label_encoder.inverse_transform(temp)[0]
            
            # publish prediction
            self.publish(MODE.ASL.name, prediction)

        # for Gesture
        elif msg.topic == self.GESTURE_REQUEST_TOPIC:
            # request payload
            resp_dict = json.loads(msg.payload)
            
            # prediction
            output = np.argmax(self.gesture_model.predict([resp_dict['raw']]), axis=-1)
            prediction = self.GESTURE[output[0]]
            
            # publish prediction
            self.publish(MODE.GESTURE.name, prediction)

        else:
            pass
        
        # for debugging purposes
        print(f'prediction = {prediction}')
        
    def publish(self, mode, prediction):
        '''
        prediction structure:
        {
            'mode': 'ASL' or 'GESTURE',
            'response': prediction
        }
        '''
        payload = json.dumps(
            {
                'mode': mode,
                'response': prediction
            }
        )
        self.client.publish(self.GESTURE_RESPONSE_TOPIC if mode == MODE.GESTURE.name else self.ASL_RESPONSE_TOPIC, payload)

    
    def disconnect(self):
        self.client.disconnect()
        self.exit = True
        print(f'==========   Disconnected to {self.id}   ==========')


class MODE(enum.Enum):
    GESTURE = 1
    ASL = 2

if __name__ == "__main__":
    # TODO: uncomment to hide warnings
    import warnings
    warnings.filterwarnings("ignore") 

    mq = myMqtt()

    try:
        mq.run()
    except KeyboardInterrupt:
        mq.disconnect()
    finally:
        print('Program exits.')

