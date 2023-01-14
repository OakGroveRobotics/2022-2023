/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "RobertAutotest2", group = "Concept")
public class RobertAutotest2 extends LinearOpMode {

    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
     private static final String TFOD_MODEL_FILE  = "ConeSleeve2.tflite";


    private static final String[] LABELS = {
            "2 Circle",
            "1 Rectangle",
            "3 Triangle"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            " AcjeWAT/////AAABmc0E7FQPHETai5ZceoUNuTV5JBz3vGpqlk57SCvmEUPEU0Fl6NkLeZNkZBXuwYgfjSLG4VsvfQXqk0jcUiA1oTLZ7LVZI+SSpyfprp6TkQWsRmJuwJPc9mEseo41D3bnsvXY/fxHHsY/CilfOEV9t0+ZEMrrpTUrdM/XkixrKUPgUsihzkVF1NO82L1eLpiYK+YXGTNf3t3wmtmZ28Tsuy39IoS3qqy+DISCnhbm56AlEBlmZ2dIeTY5r9rLFgA/xYA8v73TSMLtI70C4MPW3FfCwxOXm+CvuPxX890mxgbkhKmIRAaLLK9cKVwa6lDlsgSmHyKKXwjT9lVcyew4OzAUe4AY+UEhR4Ywc3n1KgR2";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        ElapsedTime elapsedTime = new ElapsedTime();

        Mecanum drive = new Mecanum(
                new Motor(hardwareMap, "left_front_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "right_front_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "left_rear_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "right_rear_drive", Motor.GoBILDA.RPM_223)
        );

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        elapsedTime.reset();

        if (opModeIsActive()) {
            drive.driveRobotCentric(.15,0,0);//Speed Forward
            while (opModeIsActive()) {
                if (tfod != null) {

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions(); //Declare List of type Recognition named updatedRecognitions

                    while(elapsedTime.time() < 8.25){ //Time Forward (Change this and Line 158)
                        try { //Try to invoke isEmpty() method on updatedRecognitions
                            if (updatedRecognitions != null && !(updatedRecognitions.isEmpty())) { // If updatedRecognitions isn't null and isn't empty
                                telemetry.addData("Status", "Run Time5: " + elapsedTime.toString());
                                telemetry.update();
                                break; // break out of while loop
                            }
                        }
                        catch(Exception e){
                            //Catch nullPointerException from invoking isEmpty() method on null updatedRecognition
                        }
                        updatedRecognitions = tfod.getUpdatedRecognitions();
                    }
                    while(elapsedTime.time() < 8.25){ //Time Forward (Change this and Line 143)
                        telemetry.addData("# Objects Detected1", updatedRecognitions.size());
                        telemetry.addData("Status", "Run Time: " + elapsedTime.toString());
                        telemetry.update();
                    }
                    drive.driveRobotCentric(0,0,0);
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        telemetry.addData("Status", "Run Time1: " + elapsedTime.toString());
                        telemetry.update();
                        for (Recognition recognition : updatedRecognitions) {
                            if(recognition.getLabel().equals("1 Rectangle")){
                                drive.driveRobotCentric(0,-.5,0);//Speed Left
                                sleep(1600); //Time Left
                                drive.driveRobotCentric(0,0,0);
                                break;
                            }
                            if(recognition.getLabel().equals("3 Triangle")){
                                drive.driveRobotCentric(0,.5,0);//Speed Right
                                sleep(1600); //Time Right
                                drive.driveRobotCentric(0,0,0);
                                break;
                            }
                            }

                        }
                    }
                }
            }
        }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /**
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
       // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
