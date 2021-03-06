import socket

#Messages when ending instances
END_NUMS = {
    -3 : "Failed To Open Camera",
    -2 : "Bad Receive",
    -1 : "Interrupted",
     0 : "Normal End",
}

TIME_OUT = 0.3

HOST_NAME = socket.gethostname()
LOCAL_IP = socket.gethostbyname(HOST_NAME)
TELLO_IP = "192.168.10.1"

USING_DRONE = False
