package org.firstinspires.ftc.teamcode;


import static com.arcrobotics.ftclib.hardware.motors.Motor.ZeroPowerBehavior.BRAKE;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Drivebase.Mecanum;

import java.util.ResourceBundle;

@TeleOp(name="Robert", group="Linear Opmode")
public class RobertIterative extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    boolean RobotCentric;

    Mecanum drive = new Mecanum(
            new Motor(hardwareMap, "left_front_drive", Motor.GoBILDA.RPM_223),
            new Motor(hardwareMap, "right_front_drive", Motor.GoBILDA.RPM_223),
            new Motor(hardwareMap, "left_rear_drive", Motor.GoBILDA.RPM_223),
            new Motor(hardwareMap, "right_rear_drive", Motor.GoBILDA.RPM_223)
    );

    MotorGroup arm = new MotorGroup(new Motor(hardwareMap, "arm1", Motor.GoBILDA.RPM_223), new Motor(hardwareMap, "arm2", Motor.GoBILDA.RPM_223));

    SimpleServo flipper = new SimpleServo(
            hardwareMap, "flipper", 0, 300,
            AngleUnit.DEGREES
    );

    SimpleServo claw = new SimpleServo(
            hardwareMap, "claw", 0, 180,
            AngleUnit.DEGREES
    );

    GamepadEx Control = new GamepadEx(gamepad1);

    @Override
    public void init() {

        arm.setZeroPowerBehavior(BRAKE);

        flipper.setRange(90, 250);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    public void start() {
        runtime.reset();
    }
        // run until the end of the match (driver presses STOP)
    public void loop(){

         /* double FORWARD_VEL = Math.abs(Control.getLeftY()) < .1 ? 0 : Control.getLeftY();
            double STRAFE_VEL  = Math.abs(Control.getLeftX()) < .1 ? 0 : Control.getLeftX();
            double ROTATE_VEL  = Math.abs(Control.getRightX()) < .1 ? 0 : Control.getRightX();*/

            double FORWARD_VEL = Control.getLeftY();
            double STRAFE_VEL  = Control.getLeftX();
            double ROTATE_VEL  = Control.getRightX();

            if(gamepad2.a){

            }

            double ARM_VEL = Math.pow(-Control.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + Control.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),5/3);

            boolean openclaw = Control.isDown(GamepadKeys.Button.RIGHT_BUMPER);
            boolean closeclaw = Control.isDown(GamepadKeys.Button.LEFT_BUMPER);

            boolean flip_forward = Control.isDown(GamepadKeys.Button.B);
            boolean flip_top = Control.isDown(GamepadKeys.Button.Y);
            boolean flip_rear = Control.isDown(GamepadKeys.Button.X);
            boolean flip_forward_by5 = Control.isDown(GamepadKeys.Button.DPAD_DOWN);
            boolean flip_backward_by5 = Control.isDown(GamepadKeys.Button.DPAD_UP);


            if(RobotCentric){
                drive.driveRobotCentric(FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);
            }

            if(openclaw){
                claw.turnToAngle(50);
            }
            else if(closeclaw){
                claw.turnToAngle(30);
            }

            if(flip_forward){
                flipper.rotateByAngle(.08);
            }
            else if(flip_top){
                flipper.turnToAngle(100);
            }
            else if(flip_rear){
                flipper.rotateByAngle(-.08);
            }
            else if(flip_forward_by5){
                flipper.rotateByAngle(-.05);
            }
            else if(flip_backward_by5){
                flipper.rotateByAngle(.05);
            }




            RobotCentric = Control.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) ^ RobotCentric;

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("F/S/W", "%4.2f, %4.2f %4.2f", FORWARD_VEL, STRAFE_VEL, ROTATE_VEL);
            telemetry.addData("flipper angle", "%4.4f", flipper.getPosition());
            telemetry.update();
        }
    }
