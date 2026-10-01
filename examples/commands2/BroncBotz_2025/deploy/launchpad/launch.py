import math
import threading
import time
import ntcore
import launchpad_py as launchpad
from launchpad_py import LaunchpadMiniMk3
from ntcore import NetworkTableInstance
from pygame import time
from pyvjoystick import vjoy
from pyvjoystick.vjoy import VJoyDevice


class LaunchpadMini3Controller:
    lp: LaunchpadMiniMk3 = None
    hid0: VJoyDevice = None
    hid1: VJoyDevice = None
    hid2: VJoyDevice = None
    teamnumber: int = None
    networkTables: NetworkTableInstance = None
    launchpadTable: ntcore._ntcore.NetworkTable = None
    buttons: ntcore._ntcore.IntegerArraySubscriber = None
    buttonStates: list[bool] = [False]*(9*9)
    stop: bool = False
    led_thread: threading.Thread = None

    def __init__(self, ntTable: str):
        self.printWarning()

        if launchpad.LaunchpadMiniMk3().Check(1):
            self.lp = launchpad.LaunchpadMiniMk3()
            if self.lp.Open(1, "minimk3"):
                self.startupLeds()

                while True:
                    try:
                        self.teamnumber = int(input("Please enter your team number: "))
                        break
                    except:
                        print("Not a valid teamnumber, please input again\n")

                self.hid0 = vjoy.VJoyDevice(1)
                self.hid1 = vjoy.VJoyDevice(2)
                self.hid2 = vjoy.VJoyDevice(3)

                print("Launchpad Mini Mk3")

                self.setupNetworkTableClient(ntTable)
                self.startLEDThread()

        else:
            raise Exception("Launchpad Attached is not an Mk3 or is not connected.")

    def startLEDThread(self):
        # Start LED update thread
        self.led_thread = threading.Thread(target=self.led_update_loop, daemon=True)
        self.led_thread.start()

    def get_button_num(self, row: int, col: int) -> int:
        button_num = 0
        if row == 0:
            button_num = col
        else:
            button_num = row * 9 + col
        return button_num


    def setupNetworkTableClient(self, launchpadNTKey: str):
        self.networkTables = ntcore.NetworkTableInstance.getDefault()
        self.networkTables.startClient4("launchpad")
        self.networkTables.setServerTeam(self.teamnumber)
        self.networkTables.setServer("localhost") # Sim only
        self.networkTables.startDSClient()
        self.launchpadTable = self.networkTables.getTable(launchpadNTKey)
        self.buttons = self.launchpadTable.getIntegerArrayTopic("colors").subscribe([0]*41)

    def extend_array(self, shrunk_array: list[int], original_size: int) -> list[int]:
        # Create an array of the original size to hold the extended result
        extended_array = [0] * original_size

        # Iterate through each element of the shrunk array and split the combined value
        for i in range(len(shrunk_array)):
            combined_value = shrunk_array[i]

            # Extract the lower 4 bytes and the upper 4 bytes from the combined value
            extended_array[i * 2] = combined_value & 0xFFFFFFFF  # Lower 4 bytes
            if i * 2 + 1 < original_size:
                extended_array[i * 2 + 1] = (combined_value >> 32) & 0xFFFFFFFF  # Upper 4 bytes

        return extended_array

    def updateLeds(self):
        if self.networkTables.isConnected():
            rgbarr = self.buttons.get()
            if rgbarr != 0:
                rgbarr = self.extend_array(rgbarr, 9*9+1)
                for button_num in range(9*9+1):
                    row = button_num // 9
                    col = button_num % 9 - 1
                    row -= 1 if col == -1 else 0
                    col = 8 if col == -1 else col
                    rgbhex = rgbarr[button_num]
                    # if k != [0,0,0]:
                    #     print(f"Setting color of ({row},{col})")
                    # Extract red, green, and blue components using bitwise operations
                    red = (rgbhex >> 16) & 0xFF  # Shift right by 16 bits, mask with 0xFF
                    green = (rgbhex >> 8) & 0xFF  # Shift right by 8 bits, mask with 0xFF
                    blue = rgbhex & 0xFF  # Mask with 0xFF to get last 8 bits
                    self.setLed(col, row, red, green, blue)


    def updateButtonStates(self):
        XYState = self.lp.ButtonStateXY()
        if XYState:
            row = XYState[1]
            column = XYState[0]
            # print(f"({row},{column}) is {"pressed" if [XYState[2]] != 0 else "not pressed"}.")
            # print(XYState)
            # if XYState[2] != 0:
            #     print(f'({row},{column})')
            self.buttonStates[self.get_button_num(row,column)] = (XYState[2] != 0)
            if XYState[1] == 7 and XYState[2] == 0:
                self.stop = True


    def updateHidStates(self):
        for button_num in range(len(self.buttonStates)):
            button = self.buttonStates[button_num]
            if button_num < 32:
                self.hid0.set_button(button_num+1, button)
            elif button_num < 64:
                self.hid1.set_button(button_num+1-32, button)
            else:
                self.hid2.set_button(button_num+1-64, button)


    def startupLeds(self):
        self.lp.LedAllOn(5)
        self.lp.LedCtrlString(f"{self.teamnumber}", 0, 0, 63, -1, waitms=0)
        self.lp.LedAllOn(0)


    def setLed(self, x, y, red, green, blue):
        self.lp.LedCtrlXY(x, y, red, green, blue)


    def printWarning(self):
        print("\nCurrently this program only works with the Novation Launchpad Mk3 Mini")
        print(
            "0,0 is defined as the top left of the launchpad whitespace(ONLY THE WHITESPACE IS USED), +y is down and +x is to the right from the top left white space")
        print(
            "If you want any LED pattern set by default do it with the associated Java library")
        print("Stop button will be the on the top-most button row, and to the right, should be just left of the corner")
        print("PLEASE MAKE SURE YOUR VJOY CONFIG IS CORRECT")
        print("It needs to be 3 vJoy devices, with no enabled Axes, no Force Feedback, 0 POVs, and 32 buttons on each device")
        print("Please confirm that vJoy device 1 is on driverstation usb order 1, and same with 2")
        print("Make sure this is run on the same computer as the driver station for NT and HID reasons")
        print("PLEASE USE THE ASSOCIATED JAVA CLASS FOR INTERACTION\n")


    def stopCheck(self):
        return self.stop

    def led_update_loop(self):
        """Runs updateLeds in a loop asynchronously."""
        while True:
            self.updateLeds()
            time.wait(10)  # Small delay to prevent excessive CPU usage

    def close(self):
        """Stops the LED thread and cleans up resources."""
        self.stop = True  # Signal thread to stop
        if self.led_thread.is_alive():
            self.led_thread.join()  # Wait for the thread to finish
        self.lp.LedCtrlString("BYE!", 22, 0, 63, -1, waitms=0)
        self.lp.Reset()  # turn all LEDs off
        self.lp.Close()  # close the Launchpad
        self.hid0.reset()
        self.hid1.reset()



def main():
    launchPad = LaunchpadMini3Controller("launchpad")



    # General Loop for buttons and colors
    while True:
        time.wait(1)
        launchPad.updateButtonStates()
        launchPad.updateHidStates()
        # if launchPad.stopCheck():
        #     break
        # print("looop")

    launchPad.close()

if __name__ == '__main__':
    main()