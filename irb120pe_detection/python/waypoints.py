#!/usr/bin/python3

# =============================================================================== #
#                                COPYRIGHT HERE                                   #
# =============================================================================== #

# waypoints.py
# This function returns the Robot Movement TYPE (Pose: /RobMove - Action: /Move) and the movement parameters.

# ===== IMPORT REQUIRED COMPONENTS ===== #
# Import ROS2 messages:
from geometry_msgs.msg import Pose
from ros2srrc_data.msg import Action

# =============================================================================== #
# CLASS -> waypoints:

class waypoints():

    def __init__(self):

        self.RobotPoseDict = {}

        # ========== HOME ========== #
        self.RobotPoseDict["HomePos"] = Action()
        self.RobotPoseDict["HomePos"].action = "MoveJ"
        self.RobotPoseDict["HomePos"].speed = 1.0
        self.RobotPoseDict["HomePos"].movej.joint1 = 0.0
        self.RobotPoseDict["HomePos"].movej.joint2 = -30.0
        self.RobotPoseDict["HomePos"].movej.joint3 = 30.0
        self.RobotPoseDict["HomePos"].movej.joint4 = 0.0
        self.RobotPoseDict["HomePos"].movej.joint5 = 90.0
        self.RobotPoseDict["HomePos"].movej.joint6 = 0.0

        # ========== PLACE CUBE ========== #
        # Blue Cube:
        self.RobotPoseDict["PlaceBLUE_app"] = Pose()
        self.RobotPoseDict["PlaceBLUE_app"].position.x = 0.15
        self.RobotPoseDict["PlaceBLUE_app"].position.y = 0.145
        self.RobotPoseDict["PlaceBLUE_app"].position.z = 1.111
        self.RobotPoseDict["PlaceBLUE_app"].orientation.x = 0.707
        self.RobotPoseDict["PlaceBLUE_app"].orientation.y = 0.707
        self.RobotPoseDict["PlaceBLUE_app"].orientation.z = 0.0
        self.RobotPoseDict["PlaceBLUE_app"].orientation.w = 0.0
        self.RobotPoseDict["PlaceBLUE"] = Pose()
        self.RobotPoseDict["PlaceBLUE"].position.x = 0.15
        self.RobotPoseDict["PlaceBLUE"].position.y = 0.145
        self.RobotPoseDict["PlaceBLUE"].position.z = 1.08
        self.RobotPoseDict["PlaceBLUE"].orientation.x = 0.707
        self.RobotPoseDict["PlaceBLUE"].orientation.y = 0.707
        self.RobotPoseDict["PlaceBLUE"].orientation.z = 0.0
        self.RobotPoseDict["PlaceBLUE"].orientation.w = 0.0
        # Black Cube:
        self.RobotPoseDict["PlaceBLACK_app"] = Pose()
        self.RobotPoseDict["PlaceBLACK_app"].position.x = 0.105
        self.RobotPoseDict["PlaceBLACK_app"].position.y = 0.145
        self.RobotPoseDict["PlaceBLACK_app"].position.z = 1.111
        self.RobotPoseDict["PlaceBLACK_app"].orientation.x = 0.707
        self.RobotPoseDict["PlaceBLACK_app"].orientation.y = 0.707
        self.RobotPoseDict["PlaceBLACK_app"].orientation.z = 0.0
        self.RobotPoseDict["PlaceBLACK_app"].orientation.w = 0.0
        self.RobotPoseDict["PlaceBLACK"] = Pose()
        self.RobotPoseDict["PlaceBLACK"].position.x = 0.105
        self.RobotPoseDict["PlaceBLACK"].position.y = 0.145
        self.RobotPoseDict["PlaceBLACK"].position.z = 1.08
        self.RobotPoseDict["PlaceBLACK"].orientation.x = 0.707
        self.RobotPoseDict["PlaceBLACK"].orientation.y = 0.707
        self.RobotPoseDict["PlaceBLACK"].orientation.z = 0.0
        self.RobotPoseDict["PlaceBLACK"].orientation.w = 0.0
        # White Cube:
        self.RobotPoseDict["PlaceWHITE_app"] = Pose()
        self.RobotPoseDict["PlaceWHITE_app"].position.x = 0.105
        self.RobotPoseDict["PlaceWHITE_app"].position.y = 0.100
        self.RobotPoseDict["PlaceWHITE_app"].position.z = 1.111
        self.RobotPoseDict["PlaceWHITE_app"].orientation.x = 0.707
        self.RobotPoseDict["PlaceWHITE_app"].orientation.y = 0.707
        self.RobotPoseDict["PlaceWHITE_app"].orientation.z = 0.0
        self.RobotPoseDict["PlaceWHITE_app"].orientation.w = 0.0
        self.RobotPoseDict["PlaceWHITE"] = Pose()
        self.RobotPoseDict["PlaceWHITE"].position.x = 0.105
        self.RobotPoseDict["PlaceWHITE"].position.y = 0.100
        self.RobotPoseDict["PlaceWHITE"].position.z = 1.08
        self.RobotPoseDict["PlaceWHITE"].orientation.x = 0.707
        self.RobotPoseDict["PlaceWHITE"].orientation.y = 0.707
        self.RobotPoseDict["PlaceWHITE"].orientation.z = 0.0
        self.RobotPoseDict["PlaceWHITE"].orientation.w = 0.0
        # No-sticker Cube:
        self.RobotPoseDict["PlaceCUBE_app"] = Pose()
        self.RobotPoseDict["PlaceCUBE_app"].position.x = 0.15
        self.RobotPoseDict["PlaceCUBE_app"].position.y = 0.100
        self.RobotPoseDict["PlaceCUBE_app"].position.z = 1.111
        self.RobotPoseDict["PlaceCUBE_app"].orientation.x = 0.707
        self.RobotPoseDict["PlaceCUBE_app"].orientation.y = 0.707
        self.RobotPoseDict["PlaceCUBE_app"].orientation.z = 0.0
        self.RobotPoseDict["PlaceCUBE_app"].orientation.w = 0.0
        self.RobotPoseDict["PlaceCUBE"] = Pose()
        self.RobotPoseDict["PlaceCUBE"].position.x = 0.15
        self.RobotPoseDict["PlaceCUBE"].position.y = 0.100
        self.RobotPoseDict["PlaceCUBE"].position.z = 1.08
        self.RobotPoseDict["PlaceCUBE"].orientation.x = 0.707
        self.RobotPoseDict["PlaceCUBE"].orientation.y = 0.707
        self.RobotPoseDict["PlaceCUBE"].orientation.z = 0.0
        self.RobotPoseDict["PlaceCUBE"].orientation.w = 0.0

        # ========== CHECK CUBE FACE ========== #
        # Front face:
        self.RobotPoseDict["FacePose_1"] = Pose()
        self.RobotPoseDict["FacePose_1"].position.x = 0.501
        self.RobotPoseDict["FacePose_1"].position.y = 0.525
        self.RobotPoseDict["FacePose_1"].position.z = 1.48
        self.RobotPoseDict["FacePose_1"].orientation.x = 0.0
        self.RobotPoseDict["FacePose_1"].orientation.y = 0.708
        self.RobotPoseDict["FacePose_1"].orientation.z = 0.0
        self.RobotPoseDict["FacePose_1"].orientation.w = 0.707
        # Back face:
        self.RobotPoseDict["FacePose_2"] = Action()
        self.RobotPoseDict["FacePose_2"].action = "MoveROT"
        self.RobotPoseDict["FacePose_2"].speed = 1.0
        self.RobotPoseDict["FacePose_2"].moverot.yaw = 180.0
        self.RobotPoseDict["FacePose_2"].moverot.pitch = 0.0
        self.RobotPoseDict["FacePose_2"].moverot.roll = 0.0

        # Bottom face:
        self.RobotPoseDict["FacePose_3"] = Pose()
        self.RobotPoseDict["FacePose_3"].position.x = 0.63
        self.RobotPoseDict["FacePose_3"].position.y = 0.525
        self.RobotPoseDict["FacePose_3"].position.z = 1.353
        self.RobotPoseDict["FacePose_3"].orientation.x = 0.001
        self.RobotPoseDict["FacePose_3"].orientation.y = 0.147
        self.RobotPoseDict["FacePose_3"].orientation.z = -0.003
        self.RobotPoseDict["FacePose_3"].orientation.w = 0.989
        # Top face:
        self.RobotPoseDict["FacePose_4"] = Pose()
        self.RobotPoseDict["FacePose_4"].position.x = 0.525
        self.RobotPoseDict["FacePose_4"].position.y = 0.527
        self.RobotPoseDict["FacePose_4"].position.z = 1.58
        self.RobotPoseDict["FacePose_4"].orientation.x = -0.004
        self.RobotPoseDict["FacePose_4"].orientation.y = 0.940
        self.RobotPoseDict["FacePose_4"].orientation.z = 0.002
        self.RobotPoseDict["FacePose_4"].orientation.w = 0.342

        # ========== IF STICKER -> TOP... ========== #
        # Pick cube before checking top face (1):
        self.RobotPoseDict["PickTopApp"] = Action()
        self.RobotPoseDict["PickTopApp"].action = "MoveL"
        self.RobotPoseDict["PickTopApp"].speed = 0.1
        self.RobotPoseDict["PickTopApp"].movel.x = 0.0
        self.RobotPoseDict["PickTopApp"].movel.y = 0.0
        self.RobotPoseDict["PickTopApp"].movel.z = 0.05
        self.RobotPoseDict["PickTop"] = Action()
        self.RobotPoseDict["PickTop"].action = "MoveRP"
        self.RobotPoseDict["PickTop"].speed = 0.1
        self.RobotPoseDict["PickTop"].moverp.x = 0.0
        self.RobotPoseDict["PickTop"].moverp.y = 0.0
        self.RobotPoseDict["PickTop"].moverp.z = 0.19
        self.RobotPoseDict["PickTop"].moverp.yaw = 0.0
        self.RobotPoseDict["PickTop"].moverp.pitch = -20.0
        self.RobotPoseDict["PickTop"].moverp.roll = 0.0
        # Place it on top of workspace after checking top face (2):
        self.RobotPoseDict["RePickTopApp_PREV"] = Pose()
        self.RobotPoseDict["RePickTopApp_PREV"].position.x = 0.528
        self.RobotPoseDict["RePickTopApp_PREV"].position.y = 0.527
        self.RobotPoseDict["RePickTopApp_PREV"].position.z = 1.057
        self.RobotPoseDict["RePickTopApp_PREV"].orientation.x = 0.0
        self.RobotPoseDict["RePickTopApp_PREV"].orientation.y = 0.938
        self.RobotPoseDict["RePickTopApp_PREV"].orientation.z = 0.0
        self.RobotPoseDict["RePickTopApp_PREV"].orientation.w = 0.346
        self.RobotPoseDict["RePickTop_PREV"] = Action()
        self.RobotPoseDict["RePickTop_PREV"].action = "MoveL"
        self.RobotPoseDict["RePickTop_PREV"].speed = 0.1
        self.RobotPoseDict["RePickTop_PREV"].movel.x = 0.0
        self.RobotPoseDict["RePickTop_PREV"].movel.y = 0.0
        self.RobotPoseDict["RePickTop_PREV"].movel.z = -0.03
        # Pick it back before placing on tray (3):
        self.RobotPoseDict["RePickTop"] = Action()
        self.RobotPoseDict["RePickTop"].action = "MoveRP"
        self.RobotPoseDict["RePickTop"].speed = 0.1
        self.RobotPoseDict["RePickTop"].moverp.x = 0.0
        self.RobotPoseDict["RePickTop"].moverp.y = 0.0
        self.RobotPoseDict["RePickTop"].moverp.z = 0.19
        self.RobotPoseDict["RePickTop"].moverp.yaw = 0.0
        self.RobotPoseDict["RePickTop"].moverp.pitch = 20.0
        self.RobotPoseDict["RePickTop"].moverp.roll = 0.0
        self.RobotPoseDict["RePickTopApp"] = Action()
        self.RobotPoseDict["RePickTopApp"].action = "MoveL"
        self.RobotPoseDict["RePickTopApp"].speed = 0.1
        self.RobotPoseDict["RePickTopApp"].movel.x = 0.0
        self.RobotPoseDict["RePickTopApp"].movel.y = 0.0
        self.RobotPoseDict["RePickTopApp"].movel.z = 0.05

        # ========== IF STICKER -> SIDE FACES... ========== #
        # Place cube in the middle of the workspace (1):
        self.RobotPoseDict["PlaceMidApp"] = Pose()
        self.RobotPoseDict["PlaceMidApp"].position.x = 0.65
        self.RobotPoseDict["PlaceMidApp"].position.y = 0.53
        self.RobotPoseDict["PlaceMidApp"].position.z = 1.1
        self.RobotPoseDict["PlaceMidApp"].orientation.x = 0.0
        self.RobotPoseDict["PlaceMidApp"].orientation.y = 1.0
        self.RobotPoseDict["PlaceMidApp"].orientation.z = 0.0
        self.RobotPoseDict["PlaceMidApp"].orientation.w = 0.0
        self.RobotPoseDict["PlaceMid"] = Pose()
        self.RobotPoseDict["PlaceMid"].position.x = 0.65
        self.RobotPoseDict["PlaceMid"].position.y = 0.53
        self.RobotPoseDict["PlaceMid"].position.z = 1.07
        self.RobotPoseDict["PlaceMid"].orientation.x = 0.0
        self.RobotPoseDict["PlaceMid"].orientation.y = 1.0
        self.RobotPoseDict["PlaceMid"].orientation.z = 0.0
        self.RobotPoseDict["PlaceMid"].orientation.w = 0.0
        # Pick cube before checking the side faces (2):
        self.RobotPoseDict["PickSideApp_1"] = Action()
        self.RobotPoseDict["PickSideApp_1"].action = "MoveROT"
        self.RobotPoseDict["PickSideApp_1"].speed = 1.0
        self.RobotPoseDict["PickSideApp_1"].moverot.yaw = 90.0
        self.RobotPoseDict["PickSideApp_1"].moverot.pitch = 0.0
        self.RobotPoseDict["PickSideApp_1"].moverot.roll = 0.0
        self.RobotPoseDict["PickSideApp_2"] = Action()
        self.RobotPoseDict["PickSideApp_2"].action = "MoveL"
        self.RobotPoseDict["PickSideApp_2"].speed = 0.1
        self.RobotPoseDict["PickSideApp_2"].movel.x = 0.0
        self.RobotPoseDict["PickSideApp_2"].movel.y = 0.0
        self.RobotPoseDict["PickSideApp_2"].movel.z = 0.05
        self.RobotPoseDict["PickSide"] = Action()
        self.RobotPoseDict["PickSide"].action = "MoveL"
        self.RobotPoseDict["PickSide"].speed = 0.1
        self.RobotPoseDict["PickSide"].movel.x = 0.0
        self.RobotPoseDict["PickSide"].movel.y = 0.0
        self.RobotPoseDict["PickSide"].movel.z = -0.03

        # ========== CUBE ROTATION ========== #
        self.RobotPoseDict["RotApp"] = Pose()
        self.RobotPoseDict["RotApp"].position.x = 0.50
        self.RobotPoseDict["RotApp"].position.y = 0.53
        self.RobotPoseDict["RotApp"].position.z = 1.1
        self.RobotPoseDict["RotApp"].orientation.x = 0.0
        self.RobotPoseDict["RotApp"].orientation.y = 1.0
        self.RobotPoseDict["RotApp"].orientation.z = 0.0
        self.RobotPoseDict["RotApp"].orientation.w = 0.0
        self.RobotPoseDict["RotBack"] = Action()
        self.RobotPoseDict["RotBack"].action = "MoveROT"
        self.RobotPoseDict["RotBack"].speed = 1.0
        self.RobotPoseDict["RotBack"].moverot.yaw = 180.0
        self.RobotPoseDict["RotBack"].moverot.pitch = 0.0
        self.RobotPoseDict["RotBack"].moverot.roll = 0.0
        self.RobotPoseDict["RotBack_2"] = Action()
        self.RobotPoseDict["RotBack_2"].action = "MoveROT"
        self.RobotPoseDict["RotBack_2"].speed = 1.0
        self.RobotPoseDict["RotBack_2"].moverot.yaw = -180.0
        self.RobotPoseDict["RotBack_2"].moverot.pitch = 0.0
        self.RobotPoseDict["RotBack_2"].moverot.roll = 0.0
        self.RobotPoseDict["RotPlace"] = Action()
        self.RobotPoseDict["RotPlace"].action = "MoveL"
        self.RobotPoseDict["RotPlace"].speed = 0.1
        self.RobotPoseDict["RotPlace"].movel.x = 0.0
        self.RobotPoseDict["RotPlace"].movel.y = 0.0
        self.RobotPoseDict["RotPlace"].movel.z = -0.03
        self.RobotPoseDict["RotPlace2"] = Action()
        self.RobotPoseDict["RotPlace2"].action = "MoveL"
        self.RobotPoseDict["RotPlace2"].speed = 0.1
        self.RobotPoseDict["RotPlace2"].movel.x = 0.0
        self.RobotPoseDict["RotPlace2"].movel.y = 0.0
        self.RobotPoseDict["RotPlace2"].movel.z = -0.035
        self.RobotPoseDict["RotPlace3"] = Action()
        self.RobotPoseDict["RotPlace3"].action = "MoveL"
        self.RobotPoseDict["RotPlace3"].speed = 0.1
        self.RobotPoseDict["RotPlace3"].movel.x = 0.005
        self.RobotPoseDict["RotPlace3"].movel.y = 0.0
        self.RobotPoseDict["RotPlace3"].movel.z = -0.03
        self.RobotPoseDict["RotationPick"] = Action()
        self.RobotPoseDict["RotationPick"].action = "MoveRP"
        self.RobotPoseDict["RotationPick"].speed = 0.1
        self.RobotPoseDict["RotationPick"].moverp.x = 0.0
        self.RobotPoseDict["RotationPick"].moverp.y = 0.0
        self.RobotPoseDict["RotationPick"].moverp.z = 0.19
        self.RobotPoseDict["RotationPick"].moverp.yaw = 0.0
        self.RobotPoseDict["RotationPick"].moverp.pitch = 30.0
        self.RobotPoseDict["RotationPick"].moverp.roll = 0.0
        self.RobotPoseDict["RotationPlace"] = Action()
        self.RobotPoseDict["RotationPlace"].action = "MoveRP"
        self.RobotPoseDict["RotationPlace"].speed = 0.1
        self.RobotPoseDict["RotationPlace"].moverp.x = 0.0
        self.RobotPoseDict["RotationPlace"].moverp.y = 0.0
        self.RobotPoseDict["RotationPlace"].moverp.z = 0.19
        self.RobotPoseDict["RotationPlace"].moverp.yaw = 0.0
        self.RobotPoseDict["RotationPlace"].moverp.pitch = -30.0
        self.RobotPoseDict["RotationPlace"].moverp.roll = 0.0
        self.RobotPoseDict["RotZ"] = Action()
        self.RobotPoseDict["RotZ"].action = "MoveL"
        self.RobotPoseDict["RotZ"].speed = 0.1
        self.RobotPoseDict["RotZ"].movel.x = 0.0
        self.RobotPoseDict["RotZ"].movel.y = 0.0
        self.RobotPoseDict["RotZ"].movel.z = 0.03
        self.RobotPoseDict["RotX"] = Action()
        self.RobotPoseDict["RotX"].action = "MoveL"
        self.RobotPoseDict["RotX"].speed = 0.3
        self.RobotPoseDict["RotX"].movel.x = 0.15
        self.RobotPoseDict["RotX"].movel.y = 0.0
        self.RobotPoseDict["RotX"].movel.z = 0.0

    def RobotPose(self, PoseName):

        result = self.RobotPoseDict[PoseName]
        return(result)          
            
