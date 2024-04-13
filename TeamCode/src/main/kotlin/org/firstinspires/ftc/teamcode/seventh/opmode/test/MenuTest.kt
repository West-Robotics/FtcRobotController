package org.firstinspires.ftc.teamcode.seventh.opmode.test

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.scrapmetal.quackerama.SelectionMenu
import com.scrapmetal.quackerama.SelectionMenu.Setting
import com.scrapmetal.quackerama.SelectionMenu.Option
import org.firstinspires.ftc.teamcode.seventh.robot.hardware.Globals

@Autonomous(name = "Menu Test")
class MenuTest : LinearOpMode() {
    override fun runOpMode() {
        val selectionMenu = SelectionMenu(telemetry)
        selectionMenu.addSetting(Setting("Color", listOf(
                Option("<font color=\"#FF0000\">RED</font>", Globals.Alliance.RED),
                Option("<font color=\"#0000FF\">BLUE</font>", Globals.Alliance.RED))))
        selectionMenu.addSetting(Setting("Lane", listOf(
                Option("\n" +
                        "<tt>┏━━━┓\n</tt>" +
                        "<tt>┃·\uD83E\uDD16·┃\n</tt>" +
                        "<tt>┣━━━┫\n</tt>" +
                        "<tt>┃····┃\n</tt>" +
                        "<tt>┣━━━┫\n</tt>" +
                        "<tt>┃····┃\n</tt>" +
                        "<tt>┗━━━┛</tt>", 3),
                Option("\n" +
                        "<tt>┏━━━┓\n</tt>" +
                        "<tt>┃····┃\n</tt>" +
                        "<tt>┣━━━┫\n</tt>" +
                        "<tt>┃·\uD83E\uDD16·┃\n</tt>" +
                        "<tt>┣━━━┫\n</tt>" +
                        "<tt>┃····┃\n</tt>" +
                        "<tt>┗━━━┛</tt>", 2),
                Option("\n" +
                        "<tt>┏━━━┓\n</tt>" +
                        "<tt>┃····┃\n</tt>" +
                        "<tt>┣━━━┫\n</tt>" +
                        "<tt>┃····┃\n</tt>" +
                        "<tt>┣━━━┫\n</tt>" +
                        "<tt>┃·\uD83E\uDD16·┃\n</tt>" +
                        "<tt>┗━━━┛</tt>", 1))))
        selectionMenu.addSetting(Setting("Side", listOf(
                Option("CLOSE", Globals.Alliance.RED),
                Option("FAR", Globals.Alliance.RED))))
        selectionMenu.addSetting(Setting("Delay", listOf(
                Option("0s", 0),
                Option("1s", 1),
                Option("2s", 2),
                Option("3s", 3),
                Option("4s", 4),
                Option("5s", 5))))
        val gamepad = GamepadEx(gamepad1)

        while (opModeInInit() && !isStarted) {
            gamepad.readButtons()
            selectionMenu.refresh(
                    gamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                    gamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                    gamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT),
                    gamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            telemetry.update()
        }
    }
}