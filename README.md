A python test client which demonstrates the communication with ROSBridge.
The client can be used to test JSON and BSON serialization. 
It's also possible to switch between TCP and Websockets as a communication method with ROSBridge.

The client is subject of this Pull Request:
https://github.com/RobotWebTools/rosbridge_suite/pull/257

## How to use the client with JSON and TCP
```
roslaunch rosbridge_server rosbridge_tcp.launch
rosrun rospy_tutorials add_two_ints_server
python BSONTestClient.py --json --tcp
```

## How to use the client with BSON
```
roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
rosrun rospy_tutorials add_two_ints_server
python BSONTestClient.py --bson --tcp
```

You can also replace the parameter --tcp with --websocket to use a websocket connection.

## Relevant notes:
- Tested with Python 2.7
- The client assumes that your rosbridge server is running on localhost:9090
- For demonstration purposes, the client calls the add_two_ints service. Please call "rosrun rospy_tutorials add_two_ints_server" before running this client. 
