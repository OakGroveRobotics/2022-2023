package org.firstinspires.ftc.teamcode;


import static com.arcrobotics.ftclib.hardware.motors.Motor.RunMode.PositionControl;
import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Robert", group="Competition")
public class Robert extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        SimpleServo rightOdemetry = new SimpleServo(hardwareMap, "odometry_servo_right",0, 300);
        SimpleServo leftOdemetry = new SimpleServo(hardwareMap, "odometry_servo_left",0, 300);
        SimpleServo frontOdemetry = new SimpleServo(hardwareMap, "odometry_servo_front",0, 300);

        rightOdemetry.setInverted(true);
//        leftOdemetry.setInverted(true);

        rightOdemetry.setPosition(.4);
        leftOdemetry.setPosition(.4);
        frontOdemetry.setPosition(.7);

        BeltDrive flipper = new BeltDrive(
                new SimpleServo(hardwareMap, "flipper1",0,300),
                0,
                1,
                new SimpleServo(hardwareMap, "flipper2",0,300)
        );

        //modify these
        flipper.addPosition("forward", .6 );

        flipper.addPosition("rear", .4 );

        flipper.addPosition("upward", .5 );

        int[] clawInvert = {0};

        ServoClaw claw = new ServoClaw(new SimpleServo(hardwareMap, "claw",0,300),0, 1, clawInvert, new SimpleServo(hardwareMap,"claw2", 0,300));

        claw.setClose(.3); //modify these
        claw.setOpen(.8);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        CascadedLift lift = new CascadedLift(
                new MotorGroup(
                        new Motor(hardwareMap, "arm1", Motor.GoBILDA.RPM_223),
                        new Motor(hardwareMap, "arm2", Motor.GoBILDA.RPM_223)
                ),
                1.0,
                BRAKE
        );

        lift.setInverted(true);

        lift.addPosition(0,200);
        lift.addPosition(1,500);
        lift.addPosition(2,1000);

        lift.setPositionTolerance(.001);
        lift.setVeloCoefficients(3,0,0);
        lift.setDistancePerPulse(.0137);
        lift.setRunMode(PositionControl);
        lift.setMaxPower(0.1);
        lift.resetPosition();

        double rightTriggerPrev = 0;
        double leftTriggerPrev = 0;

        int liftPosition = 0;

        GamepadEx Control = new GamepadEx(gamepad1);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (!this.isStopRequested() && this.isStarted()) {

            double FORWARD_VEL = Control.getLeftY();
            double STRAFE_VEL  = Control.getLeftX();
            double ROTATE_VEL  = Control.getRightX();

            drive.setWeightedDrivePower( new Pose2d(
                    Control.getLeftY(),
                    -Control.getLeftX(),
                    -Control.getRightX()
                )
            );

            drive.update();


            if(Control.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || Control.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                if (claw.isOpen()) {
                    claw.Close();
                } else
                    claw.Open();
            }

            if((Control.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > .5) && rightTriggerPrev < .5){
                if(liftPosition == 2){
                    lift.setTargetPosition(2);
                }
                else {
                    lift.setTargetPosition(++liftPosition);
                }
            }
            else if((Control.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .5) && leftTriggerPrev < .5){
                if(liftPosition == 0){
                    lift.setTargetPosition(0);
                }
                else {
                    lift.setTargetPosition(--liftPosition);
                }
            }

            if(!lift.atTargetPosition()){
                lift.goToPosition(.1);
            }
            else{
                lift.goToPosition(0);
            }

            if(Control.isDown(GamepadKeys.Button.B)){
                claw.Close();
                flipper.setPosition("forward");
            }
            else if(Control.isDown(GamepadKeys.Button.Y)){
                claw.Close();
                flipper.setPosition("upward");
            }
            else if(Control.isDown(GamepadKeys.Button.X)){
                claw.Close();
                flipper.setPosition("rear");
            }
            else if(Control.isDown(GamepadKeys.Button.DPAD_DOWN)){
                flipper.rotateBy(-.01);
            }
            else if(Control.isDown(GamepadKeys.Button.DPAD_UP)){
                flipper.rotateBy(.01);

            }
            Control.readButtons();

            rightTriggerPrev = Control.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            leftTriggerPrev = Control.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Claw", "%4.2f",  claw.getPos());
            telemetry.addData("arm", "%4.2f",  lift.get());
            telemetry.addData("F/S/W", "%4.2f, %4.2f %4.2f", FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);
            telemetry.addData("flipper angle", "%4.4f", flipper.getPosition());
            telemetry.update();
        }
    }}