

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Drivebase.Mecanum;
import org.firstinspires.ftc.teamcode.Odemetry.PositionTracker;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

@Autonomous(name = "RobertAutoTest2", group = "Staging", preselectTeleOp = "Robert")
public class RobertAutoTest2 extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    static final double     COUNTS_PER_MOTOR_REV    = 8192 ;    // Rev Robotics Encoder
    static final double     WHEEL_DIAMETER_INCHES   = 2.0 ;     // For figuring circumference
    static final double     DISTANCE_PER_PULSE      = (Math.PI * WHEEL_DIAMETER_INCHES) / (COUNTS_PER_MOTOR_REV);
    static final double     TRACKWIDTH              = 0.6;
    static final double     CENTER_WHEEL_OFFSET     = 0.0;



    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int Z1 = 4; //36H11 Tag 4 Goes to Zone 1
    int Z2 = 7; //36H11 Tag 7 Goes to Zone 2
    int Z3 = 11; //36H11 Tag 11 Goes to Zone 3

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {

        Motor LeftFront = new Motor(hardwareMap, "left_front_drive", Motor.GoBILDA.RPM_223);
        Motor RightFront = new Motor(hardwareMap, "right_front_drive", Motor.GoBILDA.RPM_223);
        Motor LeftRear = new Motor(hardwareMap, "left_rear_drive", Motor.GoBILDA.RPM_223);
        Motor RightRear = new Motor(hardwareMap, "right_rear_drive", Motor.GoBILDA.RPM_223);

        Mecanum drive = new Mecanum(LeftFront, RightFront, LeftRear, RightRear);

        Encoder leftSide = LeftFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        Encoder rightSide = RightFront.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        Encoder strafe = RightRear.encoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        rightSide.setDirection(Motor.Direction.REVERSE);

       HolonomicOdometry odometry = new HolonomicOdometry(
                leftSide::getDistance,
                rightSide::getDistance,
                strafe::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == Z1 || tag.id == Z2 || tag.id == Z3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");
                }

            }
            else {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if(tagOfInterest == null) {
            /*
                Detection Failure Autonomous
             */
        }
        else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */





            // e.g.
            if(tagOfInterest.pose.x <= 20) {
                // do something
            }
            else if(tagOfInterest.pose.x >= 20 && tagOfInterest.pose.x <= 50) {
                // do something else
            }
            else if(tagOfInterest.pose.x >= 50) {
                // do something else
            }
        }


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
