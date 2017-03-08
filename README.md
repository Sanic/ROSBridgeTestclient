A python test client which demonstrates the communication with ROSBridge.
The client can be used to test JSON and BSON transmission.
Please note that BSON is currently not an implemented feature in the stable branch of ROSBridge.

The client is subject of this Pull Request:
https://github.com/RobotWebTools/rosbridge_suite/pull/257

- Tested with Python 2.7
- Assumes that your rosbridge server is running on localhost:9090
- You can switch between JSON and BSON transmission by setting TEST_MODE ( see https://github.com/Sanic/ROSBridgeTestclient/blob/master/BSONTestClient.py#L17 ). *JSON is the default*.