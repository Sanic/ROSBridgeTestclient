# TCP communication adapted from https://wiki.python.org/moin/TcpCommunication#Client

import bson
import json
import time
import socket
import pprint
import websocket


print("BSON-ROSBridge Testclient")

# Setting up the constants for the different parameters
JSON = "JSON"
BSON = "BSON"
TCP = "TCP"
WEBSOCKETS = "WEBSOCKETS"

# Set the serialization method you want to test
# TEST_MODE = JSON
TEST_MODE = BSON

# Set the communication method you want to test
# COMMUNICATION_METHOD = TCP
COMMUNICATION_METHOD = WEBSOCKETS

# TCP Parameters
TCP_IP = '127.0.0.1'
TCP_PORT = 9090
BUFFER_SIZE = 4096

# Websocket Parameters
WEBSOCKET_URL = "ws://"+TCP_IP+":"+str(TCP_PORT)

def encode_message(msg):
    if TEST_MODE == JSON:
        return json.dumps(msg)
    elif TEST_MODE == BSON:
        return bson.BSON.encode(msg)

def decode_message(msg):
    if TEST_MODE == JSON:
        return json.loads(msg)
    elif TEST_MODE == BSON:
        bson_message = bson.BSON(msg)
        return bson_message.decode()


def websocket_send_wrapper(webs,data):
    if TEST_MODE == BSON:
        webs.send_binary(encode_message(data))
    else:
        webs.send(encode_message(data))

# Prepare the messages to be sent
# This includes:
#  - Advertising a topic
#  - Publishing to it
#  - Sending a service request

advertise_msg = {
  "op": "advertise",
  "topic": "/bson_test",
  "type": "std_msgs/String"
}
my_message = {
    "data" : "Hello from BSON-ROSBridge Testclient: "
}
if TEST_MODE==BSON:
    my_message["data"]+="BSON Mode via " + str(COMMUNICATION_METHOD)
elif TEST_MODE==JSON:
    my_message["data"]+="JSON Mode via " + str(COMMUNICATION_METHOD)

pub_msg = {
  "op": "publish",
  "topic": "/bson_test",
  "msg": my_message
}

call_service_params = {
    "a" : 20,
    "b" : 22
}

call_service_msg = {
  "op": "call_service",
  "service": "/add_two_ints",
  "args": call_service_params
}


if COMMUNICATION_METHOD == TCP:
    print("Testing TCP Communication")
    # TCP communication
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    
    if TEST_MODE == BSON:
        print("Sending BSON data")
        s.send(encode_message(advertise_msg))
        s.send(encode_message(pub_msg))
        s.send(encode_message(call_service_msg))
    elif TEST_MODE == JSON:
        print("Sending JSON data")
        s.send(encode_message(advertise_msg))
        time.sleep(0.1)  # Add delay to cause message to be sent in separate tcp packets
        s.send(encode_message(pub_msg))
        time.sleep(0.1)  # Add delay to cause message to be sent in separate tcp packets
        s.send(encode_message(call_service_msg))
    else:
        print("Invalid TEST_MODE")
        s.close()
        quit(1)
    
    data = s.recv(BUFFER_SIZE)
    print("Received "+ TEST_MODE +" response over "+ COMMUNICATION_METHOD +":")
    pprint.pprint( decode_message(data))
    
    s.close()
elif COMMUNICATION_METHOD == WEBSOCKETS:
    print("Testing Websocket Communication")
    ws = websocket.create_connection(WEBSOCKET_URL)
    websocket_send_wrapper(ws,advertise_msg)
    websocket_send_wrapper(ws,pub_msg)
    websocket_send_wrapper(ws,call_service_msg)

    service_response = ws.recv()
    print("Received "+ TEST_MODE +" response :")
    pprint.pprint(decode_message(service_response))
else:
    print("Invalid COMMUNICATION_METHOD")
    quit(1)
