import TelloLIBmain

instance = telloSDK()

instance.sendMessage("takeoff")

image = instance.getImage()
while(True):   
    image = instance.getImage()

instance.end()