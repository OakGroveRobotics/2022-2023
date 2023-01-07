package org.firstinspires.ftc.teamcode;


import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivebase.Mecanum;

@TeleOp(name="Mecanum", group="Linear Opmode")
public class Robert extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {


        SimpleServo flipper = new SimpleServo(
                hardwareMap, "flipper", 0, 300,
                AngleUnit.DEGREES
        );

        SimpleServo claw = new SimpleServo(
                hardwareMap, "claw", 0, 180,
                AngleUnit.DEGREES
        );

        flipper.setInverted(false);

        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "left_front_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "right_front_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "left_rear_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "right_rear_drive", Motor.GoBILDA.RPM_223)
        );

        Motor arm1 = new Motor(hardwareMap, "arm1", Motor.GoBILDA.RPM_223);
        Motor arm2 = new Motor(hardwareMap, "arm2", Motor.GoBILDA.RPM_223);

        arm1.setZeroPowerBehavior(BRAKE);
        arm2.setZeroPowerBehavior(BRAKE);

        GamepadEx Control = new GamepadEx(gamepad1);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean RobotCentric = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double FORWARD_VEL = -gamepad1.left_stick_y;
            double STRAFE_VEL  = gamepad1.left_stick_x;
            double ROTATE_VEL  = gamepad1.right_stick_x;

            double ARM_VEL = Math.pow(-Control.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + Control.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),5/3);

            boolean openclaw = Control.isDown(GamepadKeys.Button.RIGHT_BUMPER);
            boolean closeclaw = Control.isDown(GamepadKeys.Button.LEFT_BUMPER);

            boolean flip_forward = Control.isDown(GamepadKeys.Button.B);
            boolean flip_top = Control.isDown(GamepadKeys.Button.Y);
            boolean flip_rear = Control.isDown(GamepadKeys.Button.X);


            if(RobotCentric){
                drive.driveRobotCentric(FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);
            }

            arm1.set(ARM_VEL);
            arm2.set(ARM_VEL);

            if(openclaw){
                claw.turnToAngle(50);
            }
            else if(closeclaw){
                claw.turnToAngle(30);
            }

            if(flip_forward){
                flipper.turnToAngle(80);
            }
            else if(flip_top){
                flipper.turnToAngle(150);
            }
            else if(flip_rear){
                flipper.turnToAngle(255);
            }




            RobotCentric = Control.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) ^ RobotCentric;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("F/S/W", "%4.2f, %4.2f %4.2f", FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);
            telemetry.update();
        }
    }}