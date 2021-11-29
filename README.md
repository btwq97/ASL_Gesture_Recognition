Run mosquitto first.

To run server:
python3 ./app/server.py

To run client:
python3 ./app/main.py

What to expect on client side:
1) Coroutines for Gesture, ASL and MQTT to be ran in 3 different tasks.
2) asyncio would switch between these tasks in quick successions.
3) Wait for while for the devices to be connected. Especially for SensorTag since it takes while to get connected.
4) You'll see the predicted results being printed on the terminal.
5) To exit program, press CTRL + C (keyboard interrupt) --> wait for awhile and the program exits

What to expect on the server side:
1) Server with models being loaded.
2) You'll see the predicted results being printed on the terminal.
3) To exit program, press CTRL + C (keyboard interrupt) --> wait for awhile and the program exits

