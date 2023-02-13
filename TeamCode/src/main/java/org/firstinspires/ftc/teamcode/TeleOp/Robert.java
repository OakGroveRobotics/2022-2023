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
import org.firstinspires.ftc.teamcode.Gamepad.GamepadExtension;
import org.firstinspires.ftc.teamcode.Intake.ServoClaw;
import org.firstinspires.ftc.teamcode.Lift.BeltDrive;
import org.firstinspires.ftc.teamcode.Lift.CascadedLift;

@TeleOp(name="Robert", group="Competition")
public class Robert extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime runtime2 = new ElapsedTime();

    int[] clawInvert = {0};

    int armPosition = 0;

    SimpleServo flipper1, flipper2, claw1, claw2;
    Motor LeftFront, RightFront, LeftRear, RightRear, Arm1, Arm2;

    GamepadEx Control;

    @Override
    public void runOpMode() {

         flipper1   = new SimpleServo(hardwareMap, "flipper1",0,300);
         flipper2   = new SimpleServo(hardwareMap, "flipper2",0,300);

         claw1      = new SimpleServo(hardwareMap, "claw",0,300);
         claw2      = new SimpleServo(hardwareMap, "claw2",0,300);

         LeftFront  = new Motor(hardwareMap, "left_front_drive", Motor.GoBILDA.RPM_223);
         RightFront = new Motor(hardwareMap, "right_front_drive", Motor.GoBILDA.RPM_223);
         LeftRear   = new Motor(hardwareMap, "left_rear_drive", Motor.GoBILDA.RPM_223);
         RightRear  = new Motor(hardwareMap, "right_rear_drive", Motor.GoBILDA.RPM_223);

         Arm1       = new Motor(hardwareMap, "arm1", Motor.GoBILDA.RPM_223);
         Arm2       = new Motor(hardwareMap, "arm2", Motor.GoBILDA.RPM_223);



        BeltDrive flipper = new BeltDrive(
                flipper1,
                0,
                1,
                flipper2
        );

        //modify these
        flipper.addPosition("forward", 1.0 );

        flipper.addPosition("rear", 0.0 );

        flipper.addPosition("upward", .5 );


        ServoClaw claw = new ServoClaw(
                claw1,
                0.0,
                1.0,
                clawInvert,
                claw2);

        claw.setClose(0); //modify these
        claw.setOpen(1);

        Mecanum drive = new Mecanum(LeftFront, RightFront, LeftRear, RightRear);

        CascadedLift lift = new CascadedLift(
                new MotorGroup(
                        Arm1,
                        Arm2
                ),
                1.0,
                BRAKE
        );

        Control = new GamepadEx(gamepad1);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        runtime2.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double FORWARD_VEL = Control.getLeftY();
            double STRAFE_VEL  = Control.getLeftX();
            double ROTATE_VEL  = Control.getRightX();

            double ARM_VEL = Math.pow(-Control.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + Control.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),5.0/3.0); // Math for Lift Extend

            drive.driveRobotCentric(FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);


            if(Control.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                claw.Open();
            }
            else if(Control.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                claw.Close();
            }



            lift.Extend(ARM_VEL);

            if(Control.isDown(GamepadKeys.Button.B)){
                claw.Close();
                flipper.setPosition("forward");
            }
            else if(Control.wasJustPressed(GamepadKeys.Button.A)){

            }
            else if(Control.isDown(GamepadKeys.Button.X)){
                claw.Close();
                flipper.setPosition("rear");
            }
            else if(Control.isDown(GamepadKeys.Button.DPAD_DOWN)){
                flipper.rotateBy(-.0025);
            }
            else if(Control.isDown(GamepadKeys.Button.DPAD_UP)){
                flipper.rotateBy(.0025);

            }
            runtime.reset();
            runtime2.reset();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("F/S/W", "%4.2f, %4.2f %4.2f", FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);
            telemetry.addData("flipper angle", "%4.4f", flipper.getPosition());
            telemetry.addData("Loop Time","Current " + runtime.toString(), "Previous" + runtime2.toString());
            telemetry.update();
        }
    }}