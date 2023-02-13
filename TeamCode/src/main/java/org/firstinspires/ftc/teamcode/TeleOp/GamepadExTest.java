package org.firstinspires.ftc.teamcode.TeleOp;


import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drivebase.Mecanum;
import org.firstinspires.ftc.teamcode.Intake.ServoClaw;
import org.firstinspires.ftc.teamcode.Lift.BeltDrive;
import org.firstinspires.ftc.teamcode.Lift.CascadedLift;

@TeleOp(name="GamepadExTest", group="Debug")
public class GamepadExTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    GamepadEx Control;

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

            Control.readButtons();

            telemetry.addData("LeftY", Control.getLeftY());
            telemetry.addData("LeftX", Control.getLeftX());
            telemetry.addData("RightY", Control.getRightY());
            telemetry.addData("RightX", Control.getRightX());

            telemetry.addData("DpadDown Down", "%b", Control.getButton(GamepadKeys.Button.DPAD_DOWN));
            telemetry.addData("DpadDown Down just pressed", "%b", Control.wasJustPressed(GamepadKeys.Button.DPAD_DOWN));
            telemetry.addData("DpadDown Down just released", "%b", Control.wasJustReleased(GamepadKeys.Button.DPAD_DOWN));

            sleep(50);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
}