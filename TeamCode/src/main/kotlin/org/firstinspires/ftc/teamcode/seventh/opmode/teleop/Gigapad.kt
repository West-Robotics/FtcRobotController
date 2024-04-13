package org.firstinspires.ftc.teamcode.seventh.opmode.teleop

import com.arcrobotics.ftclib.command.button.Trigger
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.PI
import kotlin.math.atan2

class Gigapad(val gamepad: Gamepad) : GamepadEx(gamepad) {
    val east = Trigger { atan2(this.leftY, this.leftX) in -1.0/8* PI..<1.0/8* PI }
    val ne = Trigger { atan2(this.leftY, this.leftX) in 1.0/8* PI..<3.0/8* PI }
    val north = Trigger { atan2(this.leftY, this.leftX) in 3.0/8* PI..<5.0/8* PI }
    val nw = Trigger { atan2(this.leftY, this.leftX) in 5.0/8* PI..<7.0/8* PI }
    val west = Trigger { atan2(this.leftY, this.leftX).let { 7.0/8* PI <= it || it < -7.0/8* PI } }
    val sw = Trigger { atan2(this.leftY, this.leftX) in -7.0/8* PI..<-5.0/8* PI }
    val south = Trigger { atan2(this.leftY, this.leftX) in -5.0/8* PI..<-3.0/8* PI }
    val se = Trigger { atan2(this.leftY, this.leftX) in -3.0/8* PI..<-1.0/8* PI }

    val inc = Trigger { -this.rightY > 0.5 }
    val dec = Trigger { -this.rightY < -0.5 }

    val lTrig = Trigger { this.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.7 }
    val rTrig = Trigger { this.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.7 }
    val guide = Trigger { this.gamepad.guide }
}