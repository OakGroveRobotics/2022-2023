package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;


@Autonomous(name="RobertAutotest", group="2022", preselectTeleOp = "Robert")

public class RobertAutotest extends LinearOpMode {




    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "ConeSleeve2.tflite";


    private static final String[] LABELS = {
            "2 Circle",
            "1 Rectangle",
            "3 Triangle"
    };
    private static final String VUFORIA_KEY =
            " AcjeWAT/////AAABmc0E7FQPHETai5ZceoUNuTV5JBz3vGpqlk57SCvmEUPEU0Fl6NkLeZNkZBXuwYgfjSLG4VsvfQXqk0jcUiA1oTLZ7LVZI+SSpyfprp6TkQWsRmJuwJPc9mEseo41D3bnsvXY/fxHHsY/CilfOEV9t0+ZEMrrpTUrdM/XkixrKUPgUsihzkVF1NO82L1eLpiYK+YXGTNf3t3wmtmZ28Tsuy39IoS3qqy+DISCnhbm56AlEBlmZ2dIeTY5r9rLFgA/xYA8v73TSMLtI70C4MPW3FfCwxOXm+CvuPxX890mxgbkhKmIRAaLLK9cKVwa6lDlsgSmHyKKXwjT9lVcyew4OzAUe4AY+UEhR4Ywc3n1KgR2";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        Mecanum drive = new Mecanum(
                new Motor(hardwareMap, "left_front_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "right_front_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "left_rear_drive", Motor.GoBILDA.RPM_223),
                new Motor(hardwareMap, "right_rear_drive", Motor.GoBILDA.RPM_223)
        );

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    ArrayList Objects = new ArrayList();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        telemetry.update();

                        Objects = getObjects(updatedRecognitions, Objects);

                        Collections.sort(Objects);

                        String Destinationmode(Objects);


                        /*if (recognition.getLabel().equals("1 Rectangle")) {
                            drive.driveRobotCentric(0, 0, .5);
                            sleep(100);
                        }
                        if (recognition.getLabel().equals("2 Circle")) {
                            drive.driveRobotCentric(0, 0, -.5);
                            sleep(100);
                        }

                        if (recognition.getLabel().equals("3 Triangle")) {
                            drive.driveRobotCentric(.5, 0, 0);
                            sleep(100);
                        }*/

                       /* telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);*/
                    }
                    telemetry.update();
                }
            }
        }
    }





    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
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
        //tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

    private ArrayList getObjects(List<Recognition> updatedRecognitions, ArrayList Objects){
        if(Objects.size()< 10){
            for(Recognition recognition:updatedRecognitions){
                Objects.add(recognition.getLabel());
            }
        }
        return Objects;
    }

    private String mode( ArrayList Objects){
        Iterator iterator = Objects.iterator();
        int[] count = new int[3];
        while(iterator.hasNext()){ //This section is so ugly, I thought I was looking at the FE exam study guide.
            if(iterator.next().equals(LABELS[0])){
                count[0]++;
            }
            if(iterator.next().equals(LABELS[1])){
                count[2]++;
            }
            if(iterator.next().equals(LABELS[2])){
                count[2]++;
            }

        }
        return LABELS[Arrays.stream(count).max().getAsInt()]; //Wont work
    }


}