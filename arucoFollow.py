from geometry_msgs.msg import Twist, Vector3, Transform, Quaternion
import math
import rtde_control
import rtde_receive
import time
import rospy
from fiducial_msgs.msg import FiducialArray
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import String

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


def radians_to_degrees(euler):

    roll = euler[0] * 180/3.14
    pitch = euler[1]* 180/3.14
    yaw = euler[2]* 180/3.14

    return roll, pitch , yaw


class Marker:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.transform = Transform()
        self.eulerAngles = Vector3(0,0,0)

    def getPosition(self):
        return [self.x, self.y]

    def isSeen(self) -> bool:
        if self.x == 0.0 and self.y == 0.0:
            return False
        return True

    def isCentered(self) -> bool:
        return 375 > self.x > 345 

    def updateEulerAngles(self) -> None:
        eulerAngles = euler_from_quaternion(self.transform.rotation.x,self.transform.rotation.y,self.transform.rotation.z,self.transform.rotation.w)
        roll, pitch, yaw = radians_to_degrees(eulerAngles)
        self.eulerAngles.x = roll
        self.eulerAngles.y = pitch
        self.eulerAngles.z = yaw


class FollowingNode:
    def __init__(self) -> None:
        self.ip = "192.168.1.211"
        self.rtde_c = rtde_control.RTDEControlInterface(self.ip)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.ip)
        self.selectionVector = [0, 0, 0, 0, 0, 0]
        self.TOOL_FEATURE = 1

        self.speedLinear = 0.07
        self.speedAngular = 0.13

        self.aruco = Marker()
        self.distanceToArucoMax = 0.45
        self.distanceToArucoMin = 0.40

        self.TcpPose = self.getTcpPose()
        self.TcpLocation = Vector3(self.TcpPose[0],self.TcpPose[1],self.TcpPose[2])
        self.eulerAngles = Vector3(self.TcpPose[3],self.TcpPose[4],self.TcpPose[5])
        
        self.horizontaReachMax = 0.35
        self.horizontaReachMin = -0.175
        self.forwardBackReachMax = -0.60
        self.forwardBackReachMin = -0.9

        rospy.init_node('follow_marker')
        rospy.Subscriber("/fiducial_vertices", FiducialArray, self.updateArucoPos)
        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.updateArucoData)
    
        self.basePublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.isGoingForward = False
        self.isGoingBack = False
        self.isGoingLeft = False
        self.isGoingRight = False
        self.baseLinear = Vector3(0, 0, 0)
        self.baseAngular = Vector3(0, 0, 0)


    def updateArucoData(self, data: FiducialTransformArray) -> None:
        self.aruco.transform.translation = self.getArucoTranslation(data)
        self.aruco.transform.rotation = self.getArucoRotation(data)

    def updateArucoPos(self, data) -> None:
        data = str(data)
        axes = ['x0','y0','x1']
        pos = []
        try:
            for a in range(2):
                index1 = data.find(axes[a])
                index2 = data.find(axes[a+1])
                tmp_position = data[index1+4:index2-1]
                tmp_position = tmp_position[:tmp_position.find('.')+5]
                pos.append(float(tmp_position))

            self.aruco.x = pos[0]
            self.aruco.y = pos[1]
        except:
            self.aruco.x = 0.0
            self.aruco.y = 0.0

    def getArucoTranslation(self, data: FiducialTransformArray) -> Vector3:
        translation: Vector3 = Vector3(0,0,0)
        try:
            stringData = str(data.transforms)
            stringData = stringData.splitlines()
            xData:String = stringData[3]
            yData:String = stringData[4]
            zData:String = stringData[5]
            translation.x = float(xData.strip()[3:])
            translation.y = float(yData.strip()[3:])
            translation.z = float(zData.strip()[3:])
        except Exception as e:
            print("aruco not detected")
        return translation

    def getArucoRotation(self, data: FiducialTransformArray) -> Quaternion:
        rotation: Quaternion = Quaternion(0,0,0,0)
        try:
            stringData = str(data.transforms)
            stringData = stringData.splitlines()
            xData:String = stringData[7]
            yData:String = stringData[8]
            zData:String = stringData[9]
            wData:String = stringData[10]
            rotation.x = float(xData.strip()[3:])
            rotation.y = float(yData.strip()[3:])
            rotation.z = float(zData.strip()[3:])
            rotation.w = float(wData.strip()[3:])
        except Exception as e:
           print("aruco not detected")
        return rotation   

    def updateArmSpeed(self) -> None:
        self.rtde_c.jogStart(self.selectionVector, self.TOOL_FEATURE)

    def moveForward(self) -> None:
        self.selectionVector[2] = self.speedLinear
        
    def moveBack(self) -> None:
        self.selectionVector[2] = -self.speedLinear

    def moveLeft(self) -> None:
        self.selectionVector[0] = self.speedLinear
  
    def moveRight(self) -> None:
        self.selectionVector[0] = -self.speedLinear
    
    def moveUp(self) -> None:
        self.selectionVector[1] = self.speedLinear
   
    def moveDown(self) -> None:
        self.selectionVector[1] = -self.speedLinear

    def pitchUp(self) -> None:
        self.selectionVector[3] = -self.speedAngular

    def pitchDown(self) -> None:
        self.selectionVector[3] = self.speedAngular

    def yawLeft(self) -> None:
        self.selectionVector[4] = self.speedAngular
    
    def yawRight(self) -> None:
        self.selectionVector[4] = -self.speedAngular
    
    def brakeHorizontal(self) -> None:
        self.selectionVector[0] = 0

    def brakeVertical(self) -> None:
        self.selectionVector[1] = 0

    def brakeForwardBack(self) -> None:
        self.selectionVector[2] = 0

    def brakeAll(self) -> None:
        self.selectionVector = [0, 0, 0, 0, 0, 0]
        self.updateArmSpeed()

    def brakePitch(self) -> None:
        self.selectionVector[3] = 0

    def brakeYaw(self) -> None:
        self.selectionVector[4] = 0

    def moveArm(self) -> None:
        
        self.aruco.updateEulerAngles()

        #Cartesian Movement
        
        if self.aruco.y < 160:
            self.moveUp()
        elif self.aruco.y > 185:
            self.moveDown()
        else:
            self.brakeVertical()

        if self.aruco.x < 345 and self.TcpLocation.y < self.horizontaReachMax:
            self.moveLeft()
        elif self.aruco.x > 375 and self.TcpLocation.y > self.horizontaReachMin:
            self.moveRight()
        else:
            self.brakeHorizontal()
        
        if self.aruco.transform.translation.z > self.distanceToArucoMax and self.TcpLocation.x > self.forwardBackReachMin and self.isReachable():
            self.moveForward()
        elif self.aruco.transform.translation.z < self.distanceToArucoMin and self.TcpLocation.x < self.forwardBackReachMax:
            self.moveBack()
        else:
            self.brakeForwardBack()

        #Euler Angles Movement
        if self.isReachable():
            if 0 < self.aruco.eulerAngles.x < 165:
                self.yawLeft()
            elif 0 > self.aruco.eulerAngles.x > -165:
                self.yawRight()
            else:
                self.brakeYaw()
            
            if self.aruco.eulerAngles.y > 10:
                self.pitchDown()
            elif self.aruco.eulerAngles.y < -10:
                self.pitchUp()
            else:
                self.brakePitch()
        else:
            self.brakePitch()
            self.brakeYaw()

        self.updateArmSpeed()
        self.updateTcpPose()
        
    def getTcpPose(self) -> list:
        return self.rtde_r.getActualTCPPose()

    def updateTcpPose(self) -> None:
        self.TcpPose = self.getTcpPose()
        self.TcpLocation.x = self.TcpPose[0]
        self.TcpLocation.y = self.TcpPose[1]
        self.TcpLocation.z = self.TcpPose[2]
        roll, pitch, yaw = radians_to_degrees(self.TcpPose[-3:])
        self.eulerAngles.x = roll
        self.eulerAngles.y = pitch 
        self.eulerAngles.z = yaw

    def getTurningDirection(self) -> str:
        if self.isGoingLeft: return "Turning Left"
        elif self.isGoingRight: return "Turning Right"
        return "Not Turning"

    def getDrivingDirection(self) -> str:
        if self.isGoingForward: return "Going Forward"
        elif self.isGoingBack: return "Going Back"
        return "Not Moving forward or back"

    def updateBase(self) -> None:
        msg = Twist(self.baseLinear, self.baseAngular)
        self.basePublisher.publish(msg)

    def turnRight(self) -> None:
        self.baseAngular.z = -0.05
        self.updateBase()
        self.isGoingLeft = False
        self.isGoingRight = True

    def turnLeft(self) -> None:
        self.baseAngular.z = 0.05
        self.updateBase()
        self.isGoingLeft = True
        self.isGoingRight = False

    def driveForward(self) -> None:
        self.baseLinear.x = 0.1
        self.updateBase()
        self.isGoingForward = True
        self.isGoingBack = False
    
    def driveBack(self) -> None:
        self.baseLinear.x = -0.1
        self.updateBase()
        self.isGoingForward = False
        self.isGoingBack = True

    def baseStop(self) -> None:
        self.baseLinear.x = 0
        self.baseAngular.z = 0
        self.updateBase()
        self.isGoingBack = False
        self.isGoingForward = False
        self.isGoingLeft = False
        self.isGoingRight = False
        
    def isReachable(self) -> bool:
        return abs(self.TcpLocation.x) + abs(self.aruco.transform.translation.z) < abs(self.forwardBackReachMin) + abs(self.distanceToArucoMin)

    def moveBase(self) -> None:
        if self.aruco.isSeen():
            if self.TcpLocation.y <= self.horizontaReachMin:
                self.turnRight()
            elif self.TcpLocation.y >= self.horizontaReachMax:
                self.turnLeft()
            elif self.aruco.isCentered() and not self.isReachable():
                self.driveForward()
            elif self.TcpLocation.x >= self.forwardBackReachMax and self.isReachable():
                self.driveBack()
            else:
                self.baseStop()
        else:
            self.baseStop()



if __name__ == '__main__':

    node = FollowingNode()

    while not rospy.is_shutdown():

        if node.aruco.isSeen():
            node.moveArm()
        else:
            node.brakeAll()

        node.moveBase()

        print("------------------ARUCO-------------------\n")
        print("Aruco translation z (distance): {}\n".format(node.aruco.transform.translation.z))
        print("Aruco Euler Angles:\n{}".format(node.aruco.eulerAngles))
        print("Aruco is Reachable: {}".format(node.isReachable()))
        print("------------------ROBOT-------------------\n")
        print("ROBOT TCP LOCATION\n{}\n".format(node.TcpLocation))
        print("ROBOT ANGLES\n{}".format(node.eulerAngles))
        print("Robot is {}".format(node.getTurningDirection()))
        print("Robot is {}".format(node.getDrivingDirection()))
        time.sleep(0.1)
