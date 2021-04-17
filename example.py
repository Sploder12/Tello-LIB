import TelloLIBmain

instance = TelloLIBmain.telloSDK()

instance.sendMessage("takeoff")

image = instance.getImage()
while(True):   
    image = instance.getImage()

instance.end()