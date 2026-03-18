# Similar to falcon xbox controller
# Creating an overlay for our fancy controller

from commands2.button import CommandJoystick, Trigger


class RebuiltControlBoard:
    """
    This class is a container for our custom Rebuilt game controller/control board
    """

    def __init__(self, portA: int, portB: int):
        """
        Constructor for the RebuiltControlBoard class.
        :param portA: The port number for the first half of the console.
        :param portB: The port number for the second half of the console.
        """
        self.ConsoleA = CommandJoystick(portA)
        self.ConsoleB = CommandJoystick(portB)

    def outpost(self) -> Trigger: # upper left right
        """
        Returns the C1 designator button on the console.
        """
        return self.ConsoleA.button(11)

    def tower(self) -> Trigger:
        """
        Returns the C2 designator button on the console.
        """
        return self.ConsoleA.button(4)

    def relayLeft(self) -> Trigger:
        """
        Returns the C3 designator button on the console.

        """
        return self.ConsoleA.button(3)

    def relayAuto(self) -> Trigger:
        """
        Returns the C4 designator button on the console.
        """
        return self.ConsoleA.button(1)

    def relayRight(self) -> Trigger:
        """
        Returns the C5 designator button on the console.
        """
        return self.ConsoleA.button(2)
    
    def bump(self) -> Trigger:
        """
        Returns the C5 designator button on the console.
        """
        return self.ConsoleA.button(9)

    def bigRed(self) -> Trigger:
        """
        Returns the C6 designator button on the console.
        """
        return self.ConsoleA.button(12)

    def bigBlue(self) -> Trigger:
        """
        Returns the C6 designator button on the console.
        """
        return self.ConsoleA.button(10)

    def launchLow(self) -> Trigger:
        """
        Returns the C7 designator button on the console.
        """
        return self.ConsoleB.button(1)

    def launchMed(self) -> Trigger:
        """
        Returns the C8 designator button on the console.
        """
        return self.ConsoleB.button(2)

    def launchHigh(self) -> Trigger:
        """
        Returns the C9 designator button on the console.
        """
        return self.ConsoleB.button(3)

    def extra1(self) -> Trigger:
        """
        Returns the C10 designator button on the console.
        """
        return self.ConsoleB.button(4)

    def extra2(self) -> Trigger:
        """
        Returns the C11 designator button on the console.
        """
        return self.ConsoleB.button(11)

    def extra3(self) -> Trigger:
        """
        Returns the C12 designator button on the console.
        """
        return self.ConsoleB.button(12)

    def switch1(self) -> Trigger:
        """
        Returns the L1 designator button on the console.
        """
        return self.ConsoleB.button(8)

    def switch2(self) -> Trigger:
        """
        Returns the L2 designator button on the console.
        """
        return self.ConsoleB.button(9)

    def switch3(self) -> Trigger:
        """
        Returns the L3 designator button on the console.
        """
        return self.ConsoleB.button(10)