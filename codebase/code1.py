from controller import *
from array import *
import math
import random
import struct

I=0
wheelRadius = 0.0525
botRadius = 0.075
linSpeed = 180 #define  speed in terms of degree/sec if the speed is been decrease the delay in function() linMo must be increase or else the position sensor value will be wrong.
linSpeedRad = linSpeed*(3.141592653589793/180)
angularAccLin =  ((linSpeedRad*linSpeedRad)/(2*(180*(3.141592653589793/180)))) # ************************** = 3.2051067 nearly , i think this value should be in rotational motor acceleration.????????????????????
rotSpeed = 30
rotSpeedM = rotSpeed               
rotSpeedRad = (((rotSpeed/wheelRadius)*botRadius)*(2/3))*(3.141592653589793/180)
angularAccRot = rotSpeedRad
channel2 = 2 #set channel used by other another bot2
channel3 = 3 #set channel used by other another bot3

robot = Robot()
timeStep = int(robot.getBasicTimeStep())
tx = robot.getEmitter('tx')
rx = robot.getReceiver('rx')
rx.enable(timeStep)
cp = robot.getCompass('cp')
cp.enable(timeStep)
ds = robot.getDistanceSensor('ds')
ds.enable(timeStep)
ps1 = robot.getPositionSensor('ps1')
ps1.enable(timeStep)
ps2 = robot.getPositionSensor('ps2')
ps2.enable(timeStep)
ac = robot.getAccelerometer('ac')
ac.enable(timeStep)

TxIR = []
TxIRNames = ["emitter(0)", "emitter(1)", "emitter(2)", "emitter(3)", "emitter(4)", "emitter(5)", "emitter(6)", "emitter(7)", "emitter(8)", "emitter(9)", "emitter(10)", "emitter(11)"]

for i in range(12):
     TxIR.append(robot.getEmitter(TxIRNames[i]))
     

RxIR = []
#RxIRNames = ["receiver(0)", "receiver(11)", "receiver(10)", "receiver(9)", "receiver(8)", "receiver(7)", "receiver(6)", "receiver(5)", "receiver(4)", "receiver(3)", "receiver(2)", "receiver(1)"]   
RxIRNames = ["receiver(0)", "receiver(1)", "receiver(2)", "receiver(3)", "receiver(4)", "receiver(5)", "receiver(6)", "receiver(7)", "receiver(8)", "receiver(9)", "receiver(10)", "receiver(11)"]

for i in range(12):
     RxIR.append(robot.getReceiver(RxIRNames[i]))
     RxIR[i].enable(timeStep) 
    
Wheel = []
WheelNames = ["wheel1", "wheel2"]

for i in range(2):
     Wheel.append(robot.getMotor(WheelNames[i]))
     Wheel[i].setPosition(float('inf'))
     Wheel[i].setVelocity(0.0)                                    
              
                                           
def delay(delayTime):
    lastTime = robot.getTime()
    while (1):
          robot.step(timeStep) 
          currentTime = robot.getTime()
          if (currentTime > lastTime):
             if (currentTime - lastTime > delayTime):
                break         
 

def timeCalLin(theta): # theta is the amount of rotation required for the wheel, the unit is in radian
    tRequire = theta/linSpeedRad  
    return tRequire  

def timeCalRot(theta): # theta is the amount of rotation required for the wheel, the unit is in radian
    tRequire = (theta/(rotSpeedRad*1.51))  #extracting the actual rotSpeedRad by removing the webot constant (webot constant = 2/3) 
    return tRequire               

def thetaRot(theoDegree):# theoritical degree needed while rotation, unit is in degree
    theta = ((botRadius/wheelRadius)*theoDegree)*(3.141592653589793/180)
    return theta
           

def thetaDis(distance):
    wheelcircum = 2*3.141592653589793*wheelRadius
    theta = ((360/wheelcircum)*distance)*(3.141592653589793/180)     
    return theta
           

def thetaNBotPo(ds): #new bot position distance
    wheelcircum = 2*3.141592653589793*wheelRadius
    theta = ds*math.sqrt(3)*(360/wheelcircum)*(3.141592653589793/180)
    return theta
            

def thetaRotCor(theoDegree, acDegree): # theoDegree is in degree and acDegree is in degree, but when the values go to thetaRot func. it retus radian value.
    #firstV = getDegrees()
    #delay(0.02)
    #secondV = getDegrees()
   # if (abs(secondV - firstV) < 1 ):  #***************************** it is 1, 0 or some other value is depended on the error pecentage of the compass
    errDegree = theoDegree - acDegree 
    theta = thetaRot(errDegree)
    return theta

def rotSpeedStat(stat):
    global rotSpeed
    global rotSpeedM
    global rotSpeedRad
    global angularAccRot
    if (stat == 0):
       rotSpeed = 15    
       rotSpeedRad = (((rotSpeed/wheelRadius)*botRadius)*(2/3))*(3.141592653589793/180)
       angularAccRot = rotSpeedRad 
    else:
       rotSpeed = rotSpeedM    
       rotSpeedRad = (((rotSpeed/wheelRadius)*botRadius)*(2/3))*(3.141592653589793/180)
       angularAccRot = rotSpeedRad
       
def getDegrees(): 
  north = cp.getValues()
  rad = math.atan2(north[0], north[2])
  degree = ((rad - 1.5708)/3.141592653589793)*180.0
  if (degree < 0.0):
    degree = degree + 360.0
  return degree           
     

def degOffset(offsetVal):
    delay(1)
    acDegrees = getDegrees()
    if (acDegrees  >= offsetVal):
       newDegree = acDegrees  - offsetVal
    else:
       newDegree = (360 - offsetVal) + acDegrees 
    return newDegree      
    

def randomization(): 
    while (1):
           robot.step(timeStep) 
           inPiorVal1 = random.randint(1, 10)                   
           chaInfo = sndData(112233, inPiorVal1, 1, 1, 0)  
           inPiorVal2 = chaInfo[0][1] 
           inPiorVal3 = chaInfo[1][1]
           print ( inPiorVal1,inPiorVal2,inPiorVal3)                                 
           if (inPiorVal1 != inPiorVal2 and inPiorVal1 != inPiorVal3):
              if (inPiorVal2 != inPiorVal3):
                 return inPiorVal1, inPiorVal2, inPiorVal3
                             
   
def cwRotMo(tRequire, theoDegree, msg):   
   robot.step(timeStep) 
   arrayDegree =[]
   arrayDs = []   
   bot1DrA = []  
   bot2DrA = [] 
   arrMsg = [500,700,900]
   val = []
   message = struct.pack('i',msg)
   
   if (400 < msg and msg < 1000):
      for i in range (3):
          if (msg != arrMsg[i]):
             val.append(arrMsg[i]) 
   else:
       val.append(0)
       val.append(0)
                                 
   s=1
   if (theoDegree < rotSpeed and theoDegree > 0):
       s = rotSpeed/theoDegree
   
   t = 1
   r = int(theoDegree/rotSpeed)
   if (r < 1):
      r = 2
      t = r
  
   delay(0.02)
   arrayDegree.append(getDegrees())
   arrayDs.append(ds.getValue() + botRadius) 
   offsetVal = getDegrees()
   
   for i in range(r):   # r is the no. of steps the bot is taking to complete the theoDegree.
       Wheel[0].setAcceleration(rotSpeedRad*s)      
       Wheel[0].setVelocity(rotSpeedRad)
       Wheel[1].setAcceleration(rotSpeedRad*s)   
       Wheel[1].setVelocity(-rotSpeedRad)
       
       lastTime = robot.getTime() 
       while (1):            
             robot.step(timeStep)
             for num in range(12):                  
                 TxIR[num].send(message)      
                 if (RxIR[num].getQueueLength() > 0):                 
                    RxIRVal = (struct.unpack('i', RxIR[num].getData()))[0]                                                     
                    if (RxIRVal == val[0]):
                       q = getDegrees() - (30*num)
                       if (q<0):
                          q = q + 360                    
                       bot1DrA.append(q)                                                                          
                    if (RxIRVal == val[1]):
                       q = getDegrees() - (30*num)
                       if (q<0):
                          q = q + 360                    
                       bot2DrA.append(q)                                                                          
                    RxIR[num].nextPacket()                                
             currentTime = robot.getTime()                    
             if (currentTime > lastTime):
                if (currentTime - lastTime > (tRequire/t)):
                   break                                   
    
       Wheel[0].setVelocity(0)
       Wheel[1].setVelocity(0)
       
       delay(0.025)              #theoritically varying this delay will not effect the rotation but in we bots it does
       arrayDegree.append(getDegrees())
       arrayDs.append(ds.getValue() + botRadius)                        
  
   if (len(bot1DrA) > 24):
      bot1Dir = 0 
      for i in range (len(bot1DrA)):
          bot1Dir = bot1Dir + bot1DrA[i]
      bot1Dir = bot1Dir/len(bot1DrA)
   else:
      bot1Dir = 400  
       
   if (len(bot2DrA) > 24):
      bot2Dir = 0 
      for i in range (len(bot2DrA)):
          bot2Dir = bot2Dir + bot2DrA[i]
      bot2Dir = bot2Dir/len(bot2DrA)
   else:
      bot2Dir = 400        
   
   newDegree = degOffset(offsetVal)     
   if (newDegree > 359 or newDegree < 1 ):
      newDegree = 0  
   acDegree = (360*(int(theoDegree/360))) + newDegree    # number of complete rotation = int(theoDegree/360)
   print (bot1Dir, bot2Dir)
   return  arrayDegree, arrayDs, acDegree, bot1Dir, bot2Dir, bot1DrA, bot2DrA
    
    
def ccwRotMo(tRequire, theoDegree, msg):
   robot.step(timeStep) 
   arrayDegree =[]
   arrayDs = []   
   bot1DrA = []  
   bot2DrA = [] 
   arrMsg = [500,700,900]
   val = []
   message = struct.pack('i',msg)
   
   if (400 < msg and msg < 1000):
      for i in range (3):
          if (msg != arrMsg[i]):
             val.append(arrMsg[i])   
   else:
       val.append(0)
       val.append(0)          
   s=1
   if (theoDegree < rotSpeed and theoDegree > 0):
       s = rotSpeed/theoDegree
   
   t = 1
   r = int(theoDegree/rotSpeed)
   if (r < 1):
      r = 2
      t = r
  
   delay(0.02)
   arrayDegree.append(getDegrees())
   arrayDs.append(ds.getValue() + botRadius) 
   offsetVal = getDegrees()
   
   for i in range(r):   # r is the no. of steps the bot is taking to complete the theoDegree.
       Wheel[0].setAcceleration(rotSpeedRad*s)      
       Wheel[0].setVelocity(-rotSpeedRad)
       Wheel[1].setAcceleration(rotSpeedRad*s)   
       Wheel[1].setVelocity(rotSpeedRad)
       
       lastTime = robot.getTime() 
       while (1):            
             robot.step(timeStep)
             for num in range(12):                  
                 TxIR[num].send(message)      
                 if (RxIR[num].getQueueLength() > 0):                 
                    RxIRVal = (struct.unpack('i', RxIR[num].getData()))[0]                                                     
                    if (RxIRVal == val[0]):
                       q = getDegrees() - (30*num)
                       if (q<0):
                          q = q + 360                    
                       bot1DrA.append(q)                                                                          
                    if (RxIRVal == val[1]):
                       q = getDegrees() - (30*num)
                       if (q<0):
                          q = q + 360                    
                       bot2DrA.append(q)                                                                          
                    RxIR[num].nextPacket()                                
             currentTime = robot.getTime()                    
             if (currentTime > lastTime):
                if (currentTime - lastTime > (tRequire/t)):
                   break                                   
    
       Wheel[0].setVelocity(0)
       Wheel[1].setVelocity(0)
       
       delay(0.025)              #theoritically varying this delay will not effect the rotation but in we bots it does
       arrayDegree.append(getDegrees())
       arrayDs.append(ds.getValue() + botRadius)                        
  
   if (len(bot1DrA) > 24):
      bot1Dir = 0 
      for i in range (len(bot1DrA)):
          bot1Dir = bot1Dir + bot1DrA[i]
      bot1Dir = bot1Dir/len(bot1DrA)
   else:
      bot1Dir = 400  
       
   if (len(bot2DrA) > 24):
      bot2Dir = 0 
      for i in range (len(bot2DrA)):
          bot2Dir = bot2Dir + bot2DrA[i]
      bot2Dir = bot2Dir/len(bot2DrA)
   else:
      bot2Dir = 400        
   
   newDegree = degOffset(offsetVal)     
   if (newDegree > 359 or newDegree < 1 ):
      newDegree = 0  
   acDegree = (360*(int(theoDegree/360))) + (360 - newDegree)    # number of complete rotation = int(theoDegree/360)
   print (bot1Dir, bot2Dir)
   return  arrayDegree, arrayDs, acDegree, bot1Dir, bot2Dir, bot1DrA, bot2DrA
    
  
def rotMotion(theoDegree, rotDirec, stat, msg): #  theoDegree = value of degree of rotation   
    rotSpeedStat(stat)
    theoDegree = abs(theoDegree)
    if (theoDegree >= rotSpeed):
       theoDegreeN = rotSpeed 
    else:
       theoDegreeN = theoDegree
    
    tRequire = timeCalRot(thetaRot(theoDegreeN))  # time required for theoritical degree of rotation for the bot
   
    #rotDirec = random.randint(1, 2)
    if (rotDirec == 1):   # cw rotation
       cwRotMoInfo = cwRotMo(tRequire, theoDegree, msg) # tRequire is a array input.      
       arrayDegree = cwRotMoInfo[0]
       arrayDs = cwRotMoInfo[1]
       acDegree = cwRotMoInfo[2] 
       bot1Dir = cwRotMoInfo[3]
       bot2Dir = cwRotMoInfo[4]          
       
       while ((theoDegree - acDegree >= 2) or (theoDegree - acDegree <= -2)): #from 5 to 5 is the range of error which may exist after rotation correction
             robot.step(timeStep) 
             if (theoDegree - acDegree >= 2):
                theoDegree = abs(theoDegree - acDegree) 
                if (theoDegree >= rotSpeed):
                   theoDegreeN = rotSpeed 
                else:
                   theoDegreeN = theoDegree
                tRequire = timeCalRot(thetaRot(theoDegreeN))  # time required for theoritical degree of rotation for the bot              
                RotMoInfo = cwRotMo(tRequire,theoDegree, msg)                 
                acDegree = RotMoInfo[2]
                
             if (theoDegree - acDegree <= -2):
                theoDegree = abs(theoDegree - acDegree) 
                if (theoDegree >= rotSpeed):
                   theoDegreeN = rotSpeed 
                else:
                   theoDegreeN = theoDegree
                tRequire = timeCalRot(thetaRot(theoDegreeN))  # time required for theoritical degree of rotation for the bot                              
                RotMoInfo = ccwRotMo(tRequire,theoDegree, msg) 
                acDegree = RotMoInfo[2]
                                                            
       return arrayDegree, arrayDs, bot1Dir, bot2Dir
       
    elif (rotDirec == 2):
       ccwRotMoInfo = ccwRotMo(tRequire, theoDegree, msg) # check the array input will work or not???????????????      
       arrayDegree = ccwRotMoInfo[0]
       arrayDs = ccwRotMoInfo[1]
       acDegree = ccwRotMoInfo[2]
       bot1Dir = ccwRotMoInfo[3]
       bot2Dir = ccwRotMoInfo[4]                     
       
       while ((theoDegree - acDegree >= 2) or (theoDegree - acDegree <= -2)): #from 5 to 5 is the range of error which may exist after rotation correction
             robot.step(timeStep) 
             if (theoDegree - acDegree >= 2):
                theoDegree = abs(theoDegree - acDegree) 
                if (theoDegree >= rotSpeed):
                   theoDegreeN = rotSpeed 
                else:
                   theoDegreeN = theoDegree
                tRequire = timeCalRot(thetaRot(theoDegreeN))  # time required for theoritical degree of rotation for the bot                                            
                RotMoInfo = ccwRotMo(tRequire,theoDegree, msg) # check the array input will work or not???????????????             
                acDegree = RotMoInfo[2]
                            
             if (theoDegree - acDegree <= -2):
                theoDegree = abs(theoDegree - acDegree) 
                if (theoDegree >= rotSpeed):
                   theoDegreeN = rotSpeed 
                else:
                   theoDegreeN = theoDegree
                tRequire = timeCalRot(thetaRot(theoDegreeN))  # time required for theoritical degree of rotation for the bot                                            
                RotMoInfo = cwRotMo(tRequire,theoDegree, msg) # check the array input will work or not???????????????                
                acDegree = RotMoInfo[2]                         
      
       return arrayDegree, arrayDs, bot1Dir, bot2Dir
       
    else:
       print ("Error in given value at the function rotMotion()")
                       

def linMo(bot1Ds):
   # ps1.enable(timeStep)
   # ps2.enable(timeStep)
   # ac.enable(timeStep)
    robot.step(timeStep)  
    tRequire = timeCalLin(thetaNBotPo(bot1Ds))
    
    ps1In = ps1.getValue()   
    ps2In = ps2.getValue()
    
    In = 0
    Fi = 0
    
    acFi = []
    stat = 1
       
    Wheel[0].setAcceleration(10)      
    Wheel[0].setVelocity(linSpeedRad)
    Wheel[1].setAcceleration(10)   
    Wheel[1].setVelocity(linSpeedRad)
    lastTime = robot.getTime()
    while (1):
          robot.step(timeStep) 
          if (In == 0):
            acIn = ac.getValues()[0]
            In = 1
          currentTime = robot.getTime()
          if (currentTime > lastTime):
             if (currentTime - lastTime > tRequire):
                break
     
    Wheel[0].setVelocity(0)  
    Wheel[1].setVelocity(0) 
    while (1):
          robot.step(timeStep)                  
          acFi.append(ac.getValues()[0])
          currentTime = robot.getTime()
          if (currentTime > lastTime):
             if (currentTime - lastTime > 2):              
                break
    delay(0.5)
    ps1Fi = ps1.getValue() 
    ps2Fi = ps2.getValue() 
    
  
    for i in range (len(acFi)):
             
        if (acFi[i] >= -1*(acIn + 0.5) and acFi[i] <= -1*(acIn - 0.5)):
           break
        else:
           print ("The simulation was terminated as the bot is collide with some unmovable object") 
           psDisMov = -1
           stat = 0 
           return psDisMov, stat                   
                                  
    ps1DisMov = abs(ps1In - ps1Fi)*(180/3.141592653589793)*((2*3.141592653589793*wheelRadius)/360)   
    ps2DisMov = abs(ps2In - ps2Fi)*(180/3.141592653589793)*((2*3.141592653589793*wheelRadius)/360)     
    psDisMov = (ps1DisMov + ps2DisMov)/2
    print (psDisMov)
 #   ps1.disable()
 #   ps2.disable()
 #   ac.disable()
    return psDisMov, stat


def botDsCh(botDir, arrayDegree,msg):
     theoDegree = botDir - getDegrees()
     while (1):
            robot.step(timeStep) 
            if (theoDegree >= 0 and abs(theoDegree) > 180):
               theoDegree = 360 - theoDegree
               rotMotion(theoDegree,2,1,msg)
               botDs = ds.getValue() + (2*botRadius)
               rotMotion(theoDegree,1,1,msg)
               break
                  
            if (theoDegree >= 0 and abs(theoDegree) <= 180):
               rotMotion(theoDegree,1,1,msg)
               botDs = ds.getValue() + (2*botRadius)
               rotMotion(theoDegree,2,1,msg)   
               break
              
            if (theoDegree < 0 and abs(theoDegree) > 180):  
               theoDegree = 360 + theoDegree
               rotMotion(theoDegree,1,1,msg)
               botDs = ds.getValue() + (2*botRadius)
               rotMotion(theoDegree,2,1,msg)
               break
              
            if (theoDegree < 0 and abs(theoDegree) <= 180):
               theoDegree = abs(theoDegree) 
               rotMotion(theoDegree,2,1,msg)
               botDs = ds.getValue() + (2*botRadius)
               rotMotion(theoDegree,1,1,msg)    
               break 
     limit = (math.atan(botRadius/botDs)*(180/3.141592653589793)) + 1         
     iRange = []  
     upL = botDir + limit
     if ( upL > 359):
        upL =  upL - 360
     loL = botDir - limit
     if (loL < 0):
        loL = loL + 360
   
     for i in range (len(arrayDegree)):
         if (upL < loL):
            if (arrayDegree[i] <= upL or arrayDegree[i] >= loL):
               iRange.append(i)
         else: 
            if (arrayDegree[i] <= upL and arrayDegree[i] >= loL):  
               iRange.append(i)
     return botDs, iRange #iRange are the points generated in the map due to bot               


def mapping(msg):
    r = 0
    while (1):  
        mapInfo = rotMotion(360,1,1,msg)
        arrayDegree = mapInfo[0]
        arrayDs = mapInfo[1]
        bot1Dir = mapInfo[2]
        bot2Dir = mapInfo[3] 
          
        x=[]
        y=[]
        stat = 1        
        
        if (bot1Dir == 400 or bot2Dir == 400):
           r = r + 1
           if (r == 3):
              bot1Ds = -1
              bot2Ds = -1
              stat = 0
              return x , y, bot1Dir, bot2Dir, bot1Ds, bot2Ds, stat
           continue
           
        if (bot1Dir != 400):                   
           botDsInf = botDsCh(bot1Dir,arrayDegree,msg)       
           bot1Ds = botDsInf[0] 
           iRange = botDsInf[1]
           
           for i in range (len(iRange)):
               arrayDs[iRange[i]] = (5*botRadius) + botRadius  # dont change this line mapping value will change # greater than the measurable range                                                                                
                    
        if (bot2Dir != 400):     
           botDsInf = botDsCh(bot2Dir,arrayDegree,msg)       
           bot2Ds = botDsInf[0] 
           iRange = botDsInf[1]                      
               
           for i in range (len(iRange)):
               arrayDs[iRange[i]] = (5*botRadius) + botRadius # dont change this line mapping value will change # greater than the measurable range 
        
        for i in range (len(arrayDegree)):
          if (arrayDs[i] <= (6*botRadius)): # dont change this line mapping value will change
             x.append(round((math.sin(arrayDegree[i]*(3.141592653589793/180))*arrayDs[i]),4))
             y.append(round((math.cos(arrayDegree[i]*(3.141592653589793/180))*arrayDs[i]),4))            
        
        twBotR = 2*botRadius
        thBotR = 3*botRadius
        print(555555555555555)
        print (bot1Ds)
        print (bot2Ds)        
        print (bot1Dir)
        print (bot2Dir)
        print(666666666666666)
        
        if (abs(bot1Ds - twBotR) < thBotR or abs(bot2Ds - twBotR)  < thBotR):       
           stat = 0                   
        if (round(abs(bot1Dir - bot2Dir)) > 65):
           anBtwBot = 360 - round(abs(bot1Dir - bot2Dir))      
        else:
           anBtwBot = round(abs(bot1Dir - bot2Dir))         
        if (anBtwBot > 65 or anBtwBot < 55):
           stat = 0                 
       
        return x , y, bot1Dir, bot2Dir, bot1Ds, bot2Ds, stat 
                  

def priortization(mapInfo):   
    print (666666666666666666666666666666)    
    x = mapInfo[0]
    y = mapInfo[1]
    bot1Dir = mapInfo[2]
    bot2Dir = mapInfo[3]
    bot1Ds = mapInfo[4]
    stat = mapInfo[6]
    
    tanCenDs = (2/3)*(math.cos(30*(3.141592653589793/180))*bot1Ds)  # traingel centroid distance 
    numPo1 = 0
    
    for i in range (len(x)):
        if (math.sqrt((x[i]*x[i]) + y[i]*y[i]) <= tanCenDs):
           numPo1 = numPo1 + 1
              
    chaInfo = sndData(331122, numPo1, 1, 1, 0) 
    numPo2 = chaInfo[0][1]
    numPo3 = chaInfo[1][1]   
    print(numPo1,numPo2,numPo3)                                                                        
    if (numPo1 > numPo2 and numPo1 > numPo3):  
       priorVal1 = 1          
      
    if (numPo1 > numPo2 and numPo1 < numPo3):
       priorVal1 = 2
         
    if (numPo1 < numPo2 and numPo1 > numPo3):
       priorVal1 = 2        
         
    if (numPo1 < numPo2 and numPo1 < numPo3):
       priorVal1 = 3         
                
    if (numPo1 == numPo2 or numPo1 == numPo3):
       while (1): 
             print (33)
             robot.step(timeStep) 
             if (numPo1 == numPo2 and numPo1 == numPo3):
                priorVal1 = random.randint(1, 3)
                chaInfo = sndData(223311, priorVal1, 1, 1, 0)
                print(44) 
                priorVal2 = chaInfo[0][1]
                priorVal3 = chaInfo[1][1]                                
            
             if (numPo1 == numPo2 and numPo1 != numPo3):            
                if (numPo1 > numPo3): 
                   priorVal1 = random.randint(1, 2)
                   priorVal3 = 3
                else:
                   priorVal1 = random.randint(2, 3)
                   priorVal3 = 1 
                  
                chaInfo = sndData(223311, priorVal1, 1, 0, 0) 
                print(55)                     
                priorVal2 = chaInfo[0][1]
                                                                     
             if (numPo1 != numPo2 and numPo1 == numPo3):      
                if (numPo1 > numPo2): 
                   priorVal1 = random.randint(1, 2)
                   priorVal2 = 3
                else:
                   priorVal1 = random.randint(2, 3)
                   priorVal2 = 1
               
                chaInfo = sndData(223311, priorVal1, 0, 1, 0)
                print(66)                      
                priorVal3 = chaInfo[1][1]                                                   
            
             if (priorVal1 != priorVal2 and priorVal1 != priorVal3):               
                break   
    print ( priorVal1) 
    n = 0 
    while (1):
          print (22)
          robot.step(timeStep)           
          if (priorVal1 == 1):
             sndData(11223344, 0, 1, 1, 1)                                               
             msg = 500
             while (1):                   
                   robot.step(timeStep)                    
                   chaInfo = sndData(44112233, 0, 1, 1, 0)  
                   if (chaInfo[0][1] == 360 or chaInfo[1][1] == 360):                  
                      rotMotion(360,1,1,msg)
                   if (chaInfo[0][1] == 0 and chaInfo[1][1] == 0):
                      break                                                           
             
             chaInfo = sndData(33441122, 0, 1, 1, 0)                          
             if (chaInfo[0][1] == 1 or chaInfo[1][1] == 1):
                priorVal1 = priorVal1 + 2
                n = n + 1 
                if (n > 3):
                   stat = 0
                   print ("The simulation was terminated as there is no scope left to make a traingluar formation by these bot in this surrounding (accoroding to the running algorithm) ") 
                   return priorVal1,bot1Ds, stat
                else:
                   continue 
                                                                                                  
             return priorVal1,bot1Ds, stat            
          
          else:                                     
             botDrA = sndData(11223344, 0, 1, 1, 2)[2]
             
             if (len(botDrA) > 0):
                botDir = 0 
                for i in range (len(botDrA)):
                    botDir = botDir + botDrA[i]
                botDir = botDir/len(botDrA)
                
                dir1 = abs(botDir - bot1Dir)
                dir2 = abs(botDir - bot2Dir)
                if (dir1 > 30 or dir2 > 30):
                   if (360 - dir1 <= 30):
                      botDir = bot1Dir
                   elif (360 - dir2 <= 30):   
                      botDir = bot2Dir
                   else:
                      botDir = 400
                else:
                   stat = 0
                   print ("the triangle formation of bots are destroyed")                   
                   return priorVal1,bot1Ds, stat 
             else:
                botDir = 400     
                          
             r=0
             while (1):                 
                 robot.step(timeStep)                 
                 if (botDir != 400): 
                    chaInfo = sndData(44112233, 0, 1, 1, 0)
                    if (chaInfo[0][1] == 0 and chaInfo[1][1] == 0):
                       break                      
                 else:
                    sndData(44112233, 360, 1, 1, 0)                     
                    btDrInfo = rotMotion(360, 1, 1, 0)
                    if (btDrInfo[2] != 400 or btDrInfo[3] != 400):
                       if (btDrInfo[2] != 400):
                          botDir = btDrInfo[2]
                       else:                                 
                          botDir = btDrInfo[3]                          
                    else:
                       r = r + 1
                       if (r == 3):
                          stat = 0
                          return priorVal1, bot1Ds, stat
                       continue                            
                                                                                                                                                                 
             theoDegree = botDir - getDegrees()
             while (1):                    
                    robot.step(timeStep)  
                    if (theoDegree >= 0 and abs(theoDegree) > 180):            
                       theoDegree = 360 - theoDegree
                       rotMotion(90,2,1,0)
                       poDs = ds.getValue() 
                       rotDir = 1
                       print (10)  
                       break
              
                    if (theoDegree >= 0 and abs(theoDegree) <= 180):           
                       rotMotion(90,1,1,0)
                       poDs = ds.getValue() 
                       rotDir = 2 
                       print (11)     
                       break
                  
                    if (theoDegree < 0 and abs(theoDegree) > 180):        
                       theoDegree = 360 + theoDegree
                       rotMotion(90,1,1,0)
                       poDs = ds.getValue() 
                       rotDir = 2
                       print (12)  
                       break
              
                    if (theoDegree < 0 and abs(theoDegree) <= 180):         
                       theoDegree = abs(theoDegree) 
                       rotMotion(90,2,1,0)
                       poDs = ds.getValue() 
                       rotDir = 1   
                       print (13)     
                       break     
             
             chckDs = bot1Ds - (2*botRadius)
             if (poDs < chckDs ):
                priorVal1 = priorVal1 - 1 
                sndData(33441122, 1, 1, 1, 0)                     
                n = n + 1  
                if (n > 3):
                   stat = 0
                   print ("The simulation was terminated as there is no scope left to make a traingluar formation by these bot in this surrounding (accoroding to the running algorithm) ") 
                   return priorVal1,bot1Ds, stat
                else:
                   rotMotion(90,rotDir,1,0)
                   continue                                                    
            
             else: 
                chaInfo = sndData(33441122, 0, 1, 1, 0)                            
                if (chaInfo[0][1] == 1 or chaInfo[1][1] == 1):
                   priorVal1 = priorVal1 - 1
                   n = n + 1 
                   if (n > 3):
                      stat = 0
                      print ("The simulation was terminated as there is no scope left to make a traingluar formation by these bot in this surrounding (accoroding to the running algorithm) ") 
                      return priorVal1,bot1Ds, stat
                   else:
                      rotMotion(90,rotDir,1,0)
                      continue     
                rotMotion(30,rotDir,1,0)
                return priorVal1,bot1Ds, stat
                          
                                                                                
def sndData(Idntty, val, identified1, identified2, IRTX):          
    chaInfo1 = []
    chaInfo2 = []
    botDrA = []
    delayTime = 4
    while (1):
          robot.step(timeStep)
          if (IRTX == 1):
             for i in range (12):  
                       TxIR[i].send(struct.pack('i', 500))
                       
          if (IRTX == 2):
             for i in range (12):
                 if (RxIR[i].getQueueLength() > 0):
                    chaInfo =  (struct.unpack('i', RxIR[i].getData()))[0]
                    if (chaInfo == 500):
                       q = getDegrees() - (30*i)
                       if (q<0):
                          q = q + 360                    
                       botDrA.append(q)                                                            
       
          if (identified1 == 1):
              rx.setChannel(channel2)       
              lastTime = robot.getTime()
              while (1):
                    robot.step(timeStep) 
                    currentTime = robot.getTime()
                    tx.send(struct.pack('ii', Idntty, val))
                    if (rx.getQueueLength() > 0):
                       chaInfo1 =  (struct.unpack('ii', rx.getData()))
                       if (chaInfo1[0]  == Idntty):                                       
                          identified1 = 0 
                       rx.nextPacket()  
                    if (currentTime > lastTime):
                       if (currentTime - lastTime > delayTime):
                          break    
         
          if (identified2 == 1):
              rx.setChannel(channel3)         
              lastTime = robot.getTime()
              while (1):
                    robot.step(timeStep) 
                    currentTime = robot.getTime()
                    tx.send(struct.pack('ii', Idntty, val))                
                    if (rx.getQueueLength() > 0):                  
                       chaInfo2 =  (struct.unpack('ii', rx.getData()))
                       if (chaInfo2[0]  == Idntty):                                       
                          identified2 = 0 
                       rx.nextPacket()  
                    if (currentTime > lastTime):
                       if (currentTime - lastTime > delayTime):
                          break                                         
          if (identified1 == 0 and identified2 == 0):
             return chaInfo1, chaInfo2, botDrA
                  
            
    
def start():
    while (1):
        robot.step(timeStep)              
        inPriorVal = randomization()       
        
        if (inPriorVal[0] < inPriorVal[1] and inPriorVal[0] < inPriorVal[2]):
           msg = 500
        if (inPriorVal[0] > inPriorVal[1] and inPriorVal[0] < inPriorVal[2]):
           msg = 700   
        if (inPriorVal[0] < inPriorVal[1] and inPriorVal[0] > inPriorVal[2]):
           msg = 700
        if (inPriorVal[0] > inPriorVal[1] and inPriorVal[0] > inPriorVal[2]):   
           msg = 900
                            
        mapInfo = mapping(msg) 
        if (mapInfo[6] == 1):                  
           priorVal = priortization(mapInfo)
           print (priorVal[0],priorVal[2],1224)
        else:
           return -1   
        
        if (priorVal[0] == 1 and priorVal[2] == 1):
           print (ps1.getValue(),23434343434)
           print (priorVal[1],1111114343)
           linMo(priorVal[1])
           print (ps1.getValue(),343434343434)
           rotMotion(180,1,1,0)
                
         
             
         
         
         
print (start())