package org.firstinspires.ftc.teamcode;


import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Drivebase.Mecanum;
import org.firstinspires.ftc.teamcode.Gamepad.GamepadExtension;
import org.firstinspires.ftc.teamcode.Intake.ServoClaw;
import org.firstinspires.ftc.teamcode.Lift.BeltDrive;
import org.firstinspires.ftc.teamcode.Lift.CascadedLift;

@TeleOp(name="Robert", group="Linear Opmode")
public class Robert extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    boolean RobotCentric;

    @Override
    public void runOpMode() {

        BeltDrive flipper = new BeltDrive(
                new SimpleServo(hardwareMap, "flipper1",0,300),
                0,
                1,
                new SimpleServo(hardwareMap, "flipper2",0,300)
        );

        flipper.addPosition("forward", .6 );

        flipper.addPosition("rear", .4 );

        flipper.addPosition("upward", .5 );

        ServoClaw claw = new ServoClaw(new SimpleServo(hardwareMap, "claw",0,300),0, 1);

        Mecanum drive = new Mecanum(
                new Motor(hardwareMap, "left_front_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "right_front_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "left_rear_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "right_rear_drive", Motor.GoBILDA.RPM_223)
        );

        CascadedLift lift = new CascadedLift(
                new MotorGroup(
                        new Motor(hardwareMap, "arm1", Motor.GoBILDA.RPM_223),
                        new Motor(hardwareMap, "arm2", Motor.GoBILDA.RPM_223)
                ),
                1.0,
                BRAKE
        );

        GamepadExtension Control = new GamepadExtension(gamepad1);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double FORWARD_VEL = Control.getLeftY();
            double STRAFE_VEL  = Control.getLeftX();
            double ROTATE_VEL  = Control.getRightX();

            double ARM_VEL = Math.pow(-Control.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + Control.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),5/3); // Math for Lift Extend



            drive.driveRobotCentric(FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);


            if(Control.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                claw.Open();
            }
            else if(Control.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                claw.Close();
            }

            lift.Extend(ARM_VEL);

            if(Control.isDown(GamepadKeys.Button.B)){
                flipper.setPosition("forward");
            }
            else if(Control.isDown(GamepadKeys.Button.Y)){
                flipper.setPosition("upward");
            }
            else if(Control.isDown(GamepadKeys.Button.X)){
                flipper.setPosition("rear");
            }
            else if(Control.isDown(GamepadKeys.Button.DPAD_DOWN)){
                flipper.rotateBy(-.01);
            }
            else if(Control.isDown(GamepadKeys.Button.DPAD_UP)){
                flipper.rotateBy(.01);

            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("F/S/W", "%4.2f, %4.2f %4.2f", FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);
            telemetry.addData("flipper angle", "%4.4f", flipper.getPosition());
            telemetry.update();
        }
    }}