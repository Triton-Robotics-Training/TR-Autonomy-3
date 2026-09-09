#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tr_messages.msg import SimTeleopInput
from pynput import keyboard
import numpy
from sim_node import constants


# Holding shift turns the number row into symbols (US layout). canonical()
# reliably un-shifts letters but not these, so map them back to digits
# explicitly to keep the spin controls working under shift.
SHIFTED_DIGITS = {
    "!": "1",
    "@": "2",
    "#": "3",
    "$": "4",
    "%": "5",
    "^": "6",
    "&": "7",
    "*": "8",
    "(": "9",
    ")": "0",
}


class TeleopInput:
    def __init__(self) -> None:
        self.pitch = 0.0
        self.yaw = 0.0
        self.x_vel = 0.0
        self.y_vel = 0.0
        self.angular_vel = 0.0

    def __str__(self) -> str:
        return (
            f"{self.x_vel}, {self.y_vel}, {self.angular_vel}, {self.pitch}, {self.yaw}"
        )


class KeyboardControls(Node):
    def __init__(self):
        super().__init__("keyboard_controls")

        # TODO probably make this slower
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.primary_robot_pub = self.create_publisher(
            SimTeleopInput, "simulation/primary_robot_teleop", 10
        )
        self.secondary_robot_pub = self.create_publisher(
            SimTeleopInput, "simulation/secondary_robot_teleop", 10
        )
        self.third_robot_pub = self.create_publisher(
            SimTeleopInput, "simulation/third_robot_teleop", 10
        )
        self.fourth_robot_pub = self.create_publisher(
            SimTeleopInput, "simulation/fourth_robot_teleop", 10
        )
        self.primary_robot_teleop_input = TeleopInput()
        self.secondary_robot_teleop_input = TeleopInput()
        self.third_robot_teleop_input = TeleopInput()
        self.fourth_robot_teleop_input = TeleopInput()

        # the secondary control scheme (i/j/k/l + number keys) is reused for the
        # third and fourth robots by holding a modifier:
        #   no modifier -> secondary robot
        #   shift       -> third robot
        #   ctrl        -> fourth robot
        self.shift_pressed = False
        self.ctrl_pressed = False

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release,
        )
        self.listener.start()

    def _modifier_target(self) -> TeleopInput:
        # ctrl takes precedence over shift if both are held
        if self.ctrl_pressed:
            return self.fourth_robot_teleop_input
        if self.shift_pressed:
            return self.third_robot_teleop_input
        return self.secondary_robot_teleop_input

    def _normalized_char(self, key: keyboard.Key):
        # normalize the key so modifiers don't change which key we detect:
        #   shift+i -> "I", ctrl+i -> control char  both map back to "i"
        #   shift+1 -> "!"                           maps back to "1"
        # returns the base letter/digit char, or None for keys we don't handle
        key = self.listener.canonical(key)
        if not hasattr(key, "char") or key.char is None:
            return None
        char = SHIFTED_DIGITS.get(key.char, key.char)
        if not char.isalpha() and not char.isdigit():
            return None
        return char

    def on_press(self, key: keyboard.Key):
        if key in (keyboard.Key.shift, keyboard.Key.shift_l, keyboard.Key.shift_r):
            self.shift_pressed = True
            return
        if key in (keyboard.Key.ctrl, keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
            self.ctrl_pressed = True
            return

        char = self._normalized_char(key)
        if char is None:
            return

        # primary robot translation
        if char == "t":
            self.primary_robot_teleop_input.x_vel = constants.TELEOP_TRANSLATION_VEL_M_S
        elif char == "g":
            self.primary_robot_teleop_input.x_vel = (
                -constants.TELEOP_TRANSLATION_VEL_M_S
            )
        # positive y_vel is left
        elif char == "f":
            self.primary_robot_teleop_input.y_vel = constants.TELEOP_TRANSLATION_VEL_M_S
        elif char == "h":
            self.primary_robot_teleop_input.y_vel = (
                -constants.TELEOP_TRANSLATION_VEL_M_S
            )
        # primary robot angular rotation
        # positive angular velocity is counterclockwise
        elif char == "r":
            self.primary_robot_teleop_input.angular_vel = (
                constants.TELEOP_ANGULAR_VEL_RADS_S
            )
        elif char == "y":
            self.primary_robot_teleop_input.angular_vel = (
                -constants.TELEOP_ANGULAR_VEL_RADS_S
            )

        # secondary control scheme: routed to secondary/third/fourth robot
        # depending on whether shift (third) or ctrl (fourth) is held
        target = self._modifier_target()
        # translation
        if char == "i":
            target.x_vel = constants.TELEOP_TRANSLATION_VEL_M_S
        elif char == "k":
            target.x_vel = -constants.TELEOP_TRANSLATION_VEL_M_S
        # positive y_vel is left
        elif char == "j":
            target.y_vel = constants.TELEOP_TRANSLATION_VEL_M_S
        elif char == "l":
            target.y_vel = -constants.TELEOP_TRANSLATION_VEL_M_S
        # angular rotation
        # this is coded this way because secondary robot is used for target practice so we often want to beyblade it at different speeds
        elif char == "5":
            target.angular_vel = 0.0
        elif char == "4":
            target.angular_vel = constants.MAX_ANGULAR_VEL_RADS_S * 0.25
        elif char == "3":
            target.angular_vel = constants.MAX_ANGULAR_VEL_RADS_S * 0.5
        elif char == "2":
            target.angular_vel = constants.MAX_ANGULAR_VEL_RADS_S * 0.75
        elif char == "1":
            target.angular_vel = constants.MAX_ANGULAR_VEL_RADS_S
        elif char == "6":
            target.angular_vel = -constants.MAX_ANGULAR_VEL_RADS_S * 0.25
        elif char == "7":
            target.angular_vel = -constants.MAX_ANGULAR_VEL_RADS_S * 0.5
        elif char == "8":
            target.angular_vel = -constants.MAX_ANGULAR_VEL_RADS_S * 0.75
        elif char == "9":
            target.angular_vel = -constants.MAX_ANGULAR_VEL_RADS_S

    def on_release(self, key: keyboard.Key):
        if key in (keyboard.Key.shift, keyboard.Key.shift_l, keyboard.Key.shift_r):
            self.shift_pressed = False
            return
        if key in (keyboard.Key.ctrl, keyboard.Key.ctrl_l, keyboard.Key.ctrl_r):
            self.ctrl_pressed = False
            return

        char = self._normalized_char(key)
        if char is None:
            return

        # primary robot translation
        if char == "t":
            self.primary_robot_teleop_input.x_vel = 0.0
        elif char == "g":
            self.primary_robot_teleop_input.x_vel = 0.0
        elif char == "f":
            self.primary_robot_teleop_input.y_vel = 0.0
        elif char == "h":
            self.primary_robot_teleop_input.y_vel = 0.0
        # primary robot angular rotation
        elif char == "r":
            self.primary_robot_teleop_input.angular_vel = 0.0
        elif char == "y":
            self.primary_robot_teleop_input.angular_vel = 0.0

        # secondary control scheme translation: zero the axis on all three
        # possible targets so velocities don't get stuck if the modifier is
        # released before the movement key
        secondary_targets = (
            self.secondary_robot_teleop_input,
            self.third_robot_teleop_input,
            self.fourth_robot_teleop_input,
        )
        if char in ("i", "k"):
            for t in secondary_targets:
                t.x_vel = 0.0
        elif char in ("j", "l"):
            for t in secondary_targets:
                t.y_vel = 0.0

    def timer_callback(self):
        # print(
        #     "publishing",
        #     "primary: "
        #     self.primary_robot_teleop_input,
        #     "secondary: ",
        #     self.secondary_robot_teleop_input,
        # )
        self._publish(self.primary_robot_pub, self.primary_robot_teleop_input)
        self._publish(self.secondary_robot_pub, self.secondary_robot_teleop_input)
        self._publish(self.third_robot_pub, self.third_robot_teleop_input)
        self._publish(self.fourth_robot_pub, self.fourth_robot_teleop_input)

    def _publish(self, publisher, teleop_input: TeleopInput):
        msg = SimTeleopInput()
        msg.x_vel = teleop_input.x_vel
        msg.y_vel = teleop_input.y_vel
        msg.angular_vel = teleop_input.angular_vel
        publisher.publish(msg)


def main(args=None):
    print("running sim node keyboard controls")
    rclpy.init()
    node = KeyboardControls()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    print("shutting down sim node keyboard controls")
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()   
