# http://www.ryzerobotics.com/
# https://dl-cdn.ryzerobotics.com/downloads/tello/20180910/Tello%20SDK%20Documentation%20EN_1.3.pdf

import threading
import socket #for connecting wirelessly
import cv2 #required for video feed
import TelloLIBconsts as constant #contains constants see "TelloLIBconsts.py"

#class that contains all the data for the TelloSDK instance
class telloSDK:
    def __init__(self, port = 8889, host = ''):  #don't change the ports
        self.running = True

        self.port = port
        self.host = host

        # Create a UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.Dsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        if constant.USING_DRONE:  #for testing without drone
            self.tello_address = (constant.TELLO_IP, port)
        else:
            self.tello_address = (constant.LOCAL_IP, port)
            
        self.sock.bind((constant.LOCAL_IP, port))

        self.stats_port = 8890
        self.Dsock.bind((constant.LOCAL_IP, self.stats_port))

        self.local_video_port = 11111

        self.mutexLock = threading.Lock() #yay mutual exclusion
        self.datLock = threading.Lock()
        self.endLock = threading.Lock()

        self.startWait = threading.Condition()
        self.msgWait = threading.Condition() #cool anti-busy waiting technique

        self.data = None
        self.response = None
        self.Bframe = None

        #don't access these directly !!!!!!!!!!
        self.__pitch = None
        self.__roll = None
        self.__yaw = None
        self.__xSpeed = None
        self.__ySpeed = None
        self.__zSpeed = None
        self.__lowTemp = None
        self.__highTemp = None
        self.__TOFdist = None
        self.__height = None
        self.__battery = None
        self.__barometer = None
        self.__Mtime = None
        self.__xAccel = None
        self.__yAccel = None
        self.__zAccel = None
        #You can some touch other things but not these

        self.command_timeout = constant.TIME_OUT #timeout for commands, change to whatever

        #create recieve thread
        self.recvThread = threading.Thread(target=self.recv)
        self.recvThread.start()

        self.sendMessage("command") #needed to be in command mode
        self.sendMessage("streamon") #starts video stream

        if constant.USING_DRONE:
            self.recvStats = threading.Thread(target=self.recvDat)
            self.recvStats.start()

        self.ret = False
        if constant.USING_DRONE:
            self.telloVideo = cv2.VideoCapture("udp://@" + constant.TELLO_IP + ":" + str(self.local_video_port))
        else:
            self.telloVideo = cv2.VideoCapture("test.mp4") #used for testing when Tello not present
        self.scale = 3

        #create video thread
        self.recvVidThread = threading.Thread(target=self.recvVid)
        if not self.telloVideo.isOpened():
            self.end(-3)
        else:
            self.recvVidThread.start()
            self.startWait.acquire()
            self.startWait.wait(5)
            self.startWait.release()
            
  
    def __del__(self):
        self.sock.close()
        self.Dsock.close()
        self.telloVideo.release()
        self.running = False
        
    def recv(self):
        while self.recvThread.is_alive and self.running: 
            try:
                self.response, server = self.sock.recvfrom(3000)
                print(self.response.decode(encoding="utf-8"))
                
                self.msgWait.acquire()
                self.msgWait.notify() #tell the main thread it can wake up
                self.msgWait.release()
            except Exception as e:
                if(type(e) == ConnectionResetError):
                    print("Reseting Receive Data Connection")
                    self.sock.close()
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self.sock.bind((constant.LOCAL_IP, self.port))
                elif(type(e) == UnicodeDecodeError):
                    print("bad response")
                elif(self.running):
                    print(e)
                    self.end(-2)
    
    def recvDat(self):
        while self.recvStats.is_alive and self.running:
            try:
                response, server = self.Dsock.recvfrom(3000)
                #print(response)
                self.datLock.acquire()
                self.data = response

                #dumbest parsing possible
                splitonce = response.decode().split(';') #split the response into seperate strings for each variable
                splittwice = []
                for string in splitonce:
                    splittwice.append(string.split(':')) #split the split response into pairs of variable name and data
            
                self.__pitch = int(splittwice[0][1])
                self.__roll = int(splittwice[1][1])
                self.__yaw = int(splittwice[2][1])
                self.__xSpeed = int(splittwice[3][1])
                self.__ySpeed = int(splittwice[4][1])
                self.__zSpeed = int(splittwice[5][1])
                self.__lowTemp = int(splittwice[6][1])
                self.__highTemp = int(splittwice[7][1])
                self.__TOFdist = int(splittwice[8][1])
                self.__height = int(splittwice[9][1])
                self.__battery = int(splittwice[10][1])
                self.__barometer = float(splittwice[11][1])
                self.__Mtime = float(splittwice[12][1])
                self.__xAccel = float(splittwice[13][1])
                self.__yAccel = float(splittwice[14][1])
                self.__zAccel = float(splittwice[15][1])

                self.datLock.release()
            except Exception as e:
                self.datLock.release()
                if(type(e) == ConnectionResetError):
                    print("Reseting Data Receive Data Connection")
                    self.sock.close()
                    self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    self.sock.bind((constant.LOCAL_IP, self.port))

                elif(self.running):
                    print("Data retrieve error: " + str(e))
                    self.end(-2)
                
    def getRawDat(self):
        self.datLock.acquire()
        tmp = self.data
        self.datLock.release()
        return tmp

    #returns a dictionary of all the data
    def getDat(self):
        self.datLock.acquire()
        out = {"pitch":self.__pitch, "roll":self.__roll, "yaw":self.__yaw, #attitude
                "xSpeed":self.__xSpeed, "ySpeed":self.__ySpeed, "zSpeed":self.__zSpeed, #speeds
                "lowestTemp":self.__lowTemp, "highestTemp":self.__highTemp, "barometer":self.__barometer, #environment data
                "TOF":self.__TOFdist, "battery%":self.__battery, "motorTime":self.__Mtime,"height":self.__height, #flight data
                "xAccel":self.__xAccel, "yAccel":self.__yAccel, "zAccel":self.__zAccel} #acceleration data
        self.datLock.release()
        return out


    def recvVid(self):
        """
        Runs as a thread, sets self.Bframe to the most recent frame Tello captured.
        """
        while self.recvVidThread.is_alive and self.running:
            try:
                self.ret, frame = self.telloVideo.read()

                if(self.ret):
                    #Prevents writing an image while reading
                    if(self.mutexLock.acquire(False)): #blocking is disabled so it will write the latest frames instead of waiting with an old frames
                        height , width , layers =  frame.shape
                        new_h=int(height/self.scale)
                        new_w=int(width/self.scale)
                        self.Bframe = cv2.resize(frame, (new_w, new_h)) #resizes image
                        #cv2.imwrite("opt.png", self.Bframe) #creates an image of what is being processed (very slow, use only for testing)
                        self.mutexLock.release()

                        self.startWait.acquire()
                        self.startWait.notify()
                        self.startWait.release()

            except Exception as e:
                print(str(e))
                self.end(-2)
        self.telloVideo.release()
    
    def getImage(self):
        self.mutexLock.acquire()
        frame = self.Bframe
        self.mutexLock.release()
        return frame

    #returns -1 if failed, 1 is sucessful
    def sendMessage(self, msg):
        if self.running:
            try:
                if not msg:
                    return -1  

                if 'end' in msg:
                    self.end()
                    return 1

                # Send data
                msg = msg.encode(encoding="utf-8") 
                data = self.sock.sendto(msg, self.tello_address) #returns number of bytes sent

                #A non busy wait for response
                if(self.response is None):
                    self.msgWait.acquire()
                    self.msgWait.wait(self.command_timeout)
                    self.msgWait.release()                

                #using the same if condition may seem redundant but response changes inbetween them thanks to multithreading
                if self.response is None:
                    response = 'none_response'
                else:
                    try:
                        response = self.response.decode('utf-8')
                    except Exception as e:
                        print("Response couldn't be decoded")
                        response = 'none_response'

                self.response = None

                return response
            except KeyboardInterrupt:
                self.end(-1)
                return -1

    def end(self, errorNum = 0):
        self.endLock.acquire() #prevents several threads using end() at the same time
        if self.running:
            self.sendMessage("land") #lands the drone, very important
            self.running = False
            if(self.recvThread.is_alive):
                self.recvThread.join
            self.sock.close()
            if(self.recvVidThread.is_alive):
                self.recvThread.join
            #prints the exit code
            print("Ended Tello: " + constant.END_NUMS.get(errorNum, "ERROR NUM DOES NOT EXIST: "+str(errorNum)))
        else:
            print("Attempted To End Already Ended Tello Instance With Code " + str(errorNum)) #this usually gets triggered, don't worry about it
        self.endLock.release()
