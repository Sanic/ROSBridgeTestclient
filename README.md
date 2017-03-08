A python test client which demonstrates the communication with ROSBridge.
The client can be used to test JSON and BSON transmission.
Please note that BSON is currently not an implemented feature in the stable branch of ROSBridge.

The client is subject of this Pull Request:
https://github.com/RobotWebTools/rosbridge_suite/pull/257

## How to use the client with JSON
```
roslaunch rosbridge_server rosbridge_tcp.launch
rosrun rospy_tutorials add_two_ints_server
python BSONTestClient.py
```

## How to use the client with BSON
```
roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
rosrun rospy_tutorials add_two_ints_server
# Edit BSONTestClient.py and change TEST_MODE to BSON
python BSONTestClient.py
```

##Relevant notes:
- Tested with Python 2.7
- The client assumes that your rosbridge server is running on localhost:9090
- You can switch between JSON and BSON transmission by setting TEST_MODE ( see https://github.com/Sanic/ROSBridgeTestclient/blob/master/BSONTestClient.py#L16 ). Please note: **JSON is the default**.
- For demonstration purposes, the client calls the add_two_ints service. Please call "rosrun rospy_tutorials add_two_ints_server" before running this client. 
