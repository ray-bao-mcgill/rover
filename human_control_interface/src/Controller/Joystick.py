#!/usr/bin/env python3

import pygame
import time
import rospy
from human_control_interface.msg import Joystick_input
from arm_control.msg import ProcessedControllerInput

class Node_Joystick():
    """Gets joystick data and publishes the data to the joystick_data topic

    Topic data : Float32MultiArray
    Data:
            Input   | Possible Value
        ------------------------------
        Button 1    :   0 or 1
        Button 2    :   0 or 1
        Button 3    :   0 or 1
        Button 4    :   0 or 1
        Button 5    :   0 or 1
        Button 6    :   0 or 1
        Button 7    :   0 or 1
        Button 8    :   0 or 1
        Button 9    :   0 or 1
        Button 10   :   0 or 1
        Button 11   :   0 or 1
        Button 12   :   0 or 1
        Axis 1      : Range [-1, 1]
        Axis 2      : Range [-1, 1]
        Axis 3      : Range [-1, 1]
        Axis 4      : Range [-1, 1]
        Hat X axis  : -1, 0, or 1
        Hat Y axis  : -1, 0, or 1
    """
    def __init__(self):
        # Initialize Joystick
        try:
            self.joystick = Joystick()
        except AssertionError as error:
            rospy.logerr(str(error))

        # Initialize ROS
        rospy.init_node("joystick", anonymous=False)
        self.joystick_publisher = rospy.Publisher("joystick_data", Joystick_input, queue_size=1)
        self.arm_publisher = rospy.Publisher("processed_arm_controller_input", ProcessedControllerInput, queue_size=1)

        self.prevB3 = 0
        self.prevB4 = 0
        self.modeState = False
        self.clawState = False

        # Start Node
        self.run()

    def run(self):
        """Runs the node loop for getting and updating the joystick information for user control
        """
        while not rospy.is_shutdown():
            if rospy.is_shutdown():
                exit()
            try:
                self.joystick.update()
                msg = Joystick_input()

                # Transfer Data into msg
                msg.B1 = self.joystick.data.b1
                msg.B2 = self.joystick.data.b2
                msg.B3 = self.joystick.data.b3
                msg.B4 = self.joystick.data.b4
                msg.B5 = self.joystick.data.b5
                msg.B6 = self.joystick.data.b6
                msg.B7 = self.joystick.data.b7
                msg.B8 = self.joystick.data.b8
                msg.B9 = self.joystick.data.b9
                msg.B10 = self.joystick.data.b10
                msg.B11 = self.joystick.data.b11
                msg.B12 = self.joystick.data.b12
                msg.A1 = self.joystick.data.a1
                msg.A2 = self.joystick.data.a2
                msg.A3 = self.joystick.data.a3
                msg.A4 = self.joystick.data.a4
                msg.Hat_X = self.joystick.data.hat_x
                msg.Hat_Y = self.joystick.data.hat_y

                self.joystick_publisher.publish(msg)

                arm_ctrl = ProcessedControllerInput()
                arm_ctrl.X_dir = msg.A2**2
                if msg.A2 < 0:
                    arm_ctrl.X_dir = -1 * arm_ctrl.X_dir
                arm_ctrl.Y_dir = msg.A1**2
                if msg.A1 > 0:
                    arm_ctrl.Y_dir = -1 * arm_ctrl.Y_dir
                arm_ctrl.Z_dir = msg.A3**2
                if msg.A3 < 0:
                    arm_ctrl.Z_dir = -1 * arm_ctrl.Z_dir

                if self.risingEdge(msg.B3, self.prevB3):
                    self.modeState = True
                else:
                    self.modeState = False

                if self.risingEdge(msg.B4, self.prevB4):
                    self.clawState = True
                else:
                    self.clawState = False

                arm_ctrl.ModeChange = self.modeState
                arm_ctrl.ClawOpen   = self.clawState

                self.prevB3 = msg.B3
                self.prevB4 = msg.B4

                self.arm_publisher.publish(arm_ctrl)

                time.sleep(0.01)
            except Exception as error:
                rospy.logerr(str(error))
        exit()

    def risingEdge(self, prevSignal, nextSignal):
        if prevSignal < nextSignal:
            return True
        else: 
            return False


class Joystick():
    """Driver to get the event inputs from the Logitech Extreme 3D Pro Joystick
    """
    def __init__(self):
        """Constructor for the Joystick Driver

        Raises
        --------
            AssertionError
                Joystick wasn't able to be initialized
        """
        # Data container for the joystick input data
        self.data = JoystickData()

        # Initialize Joystick
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 1:
            # Take the only Joystick available
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
        else:
            # Either no Joystick found or multiple detected (currently unsupported)
            self.controller = None

        if self.controller is None:
            raise AssertionError("Joystick not initialized properly, make sure you have one connected")

    def update(self):
        """Gets the latest data from the joystick from event information received from the joystick
        """
        for an_event in pygame.event.get():
            try:
                # Get event information
                if an_event.type == pygame.JOYBUTTONDOWN or an_event.type == pygame.JOYBUTTONUP:
                    self.data.b1 = self.controller.get_button(0)
                    self.data.b2 = self.controller.get_button(1)
                    self.data.b3 = self.controller.get_button(2)
                    self.data.b4 = self.controller.get_button(3)
                    self.data.b5 = self.controller.get_button(4)
                    self.data.b6 = self.controller.get_button(5)
                    self.data.b7 = self.controller.get_button(6)
                    self.data.b8 = self.controller.get_button(7)
                    self.data.b9 = self.controller.get_button(8)
                    self.data.b10 = self.controller.get_button(9)
                    self.data.b11 = self.controller.get_button(10)
                    self.data.b12 = self.controller.get_button(11)
                elif an_event.type == pygame.JOYAXISMOTION:
                    self.data.a1 = self.controller.get_axis(0)
                    self.data.a2 = -1 * self.controller.get_axis(1)
                    self.data.a3 = self.controller.get_axis(2)
                    self.data.a4 = -1 * self.controller.get_axis(3)
                else:
                    self.data.hat = self.controller.get_hat(0)

                    self.data.hat_x = self.data.hat[0]
                    self.data.hat_y = self.data.hat[1]

            except pygame.error:
                pass
            finally:
                pass

    def printData(self):
        """Sends copy of joystick data to the standard output (sys.out)
        """
        data = ''
        # Buttons
        data += f"B1 {self.data.b1}| "
        data += f"B2 {self.data.b2}| "
        data += f"B3 {self.data.b3}| "
        data += f"B4 {self.data.b4}| "
        data += f"B5 {self.data.b5}| "
        data += f"B6 {self.data.b6}| "
        data += f"B7 {self.data.b7}| "
        data += f"B8 {self.data.b8}| "
        data += f"B9 {self.data.b9}| "
        data += f"B10 {self.data.b10}| "
        data += f"B11 {self.data.b11}| "
        data += f"B12 {self.data.b12}| "

        # Axis
        data += f"A1 {self.data.a1}| "
        data += f"A2 {self.data.a2}| "
        data += f"A3 {self.data.a3}| "
        data += f"A4 {self.data.a4}| "

        # Hat
        data += f"HAT {self.data.hat}"
        
        print(data)

class JoystickData():
    """Object containing mapping of the Logitech Extreme 3D Pro
    """
    def __init__(self):
        # Buttons
        self.b1 = 0
        self.b2 = 0
        self.b3 = 0
        self.b4 = 0
        self.b5 = 0
        self.b6 = 0
        self.b7 = 0
        self.b8 = 0
        self.b9 = 0
        self.b10 = 0
        self.b11 = 0
        self.b12 = 0

        # Axis
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0

        # Hat
        self.hat = (0, 0)
        self.hat_x = 0
        self.hat_y = 0


if __name__ == "__main__":
    driver = Node_Joystick()
    rospy.spin()