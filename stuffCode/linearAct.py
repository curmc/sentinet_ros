class Data:
    """
        variable types of the class:
            posX - The x position of the robot.
            posY - The y position of the robot.
            linVel - The linear velocity of the robot.
            angVel - The angular velocity of the robot.
            accel - The acceleration of the robot.
            type - The distinction of what type of data that this Data holds.
    """
    def __init__(self,dataA,dataB,type):
        self.posX = 0;
        self.posY = 0;
        self.linVel = 0;
        self.angVel = 0;
        self.accel = 0;
        if type == 0:
            self.posX = dataA
            self.posY = dataB
        elif type == 1:
            self.linVel = dataA
            self.angVel = dataB
        else:
            self.accel = dataA

        self.type = type;
    
    def getData(self):
        return self.posX, self.posY, self.linVel, self.angVel, self.accel
    
    def setDataPosX(self,x):
        self.posX = x
    
    def setDataPosY(self,y):
        self.posY = y

    def setDataLinVel(self,lin):
        self.linVel = lin

    def setDataAngVel(self,vel):
        self.angVel = vel

    def setDataAccel(self,axl):
        self.accel = axl

class LinearActuator:
    """
    This be the one that adds everything up together.
    """
    def __init__(self,Data):
        self.data = Data
    
    def averageData(self):
        relevantData = self.data.getData()
        sum = (relevantData[0]+relevantData[1]+relevantData[2]+relevantData[3]+relevantData[4])/6
        print(sum)

data = Data(8,8,1)

data.getData()

linAct = LinearActuator(data)
linAct.averageData()