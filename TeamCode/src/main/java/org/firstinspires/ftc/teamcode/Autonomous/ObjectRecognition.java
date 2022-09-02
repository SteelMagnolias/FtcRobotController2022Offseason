package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// these are our imports to get the object recognition to work.

@Autonomous(name = "ObjectRecognition", group = "IterativeOpMode")

public class ObjectRecognition extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    // this is where we can find the preset models

    private static final String[] LABELS {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    // these are labels that can be used to define what items might be seen.

    private static final String VUFORIA_KEY = "Ae/tYNP/////AAABmWJ3jgvBrksYtYG8QcdbeqRWGQWezSnxje7FgEIzwTeFQ1hZ42y6YmaQ0h5p7aqN9x+q1QXf2zRRrh1Pxln3C2cR+ul6r9mHwHbTRgd3jyggk8tzc/ubgaPBdn1q+ufcYqCk6tqj7t8JNYM/UHLZjtpSQrr5RNVs227kQwBoOx6l4MLqWL7TCTnE2vUjgrHaEW1sP1hBsyf1D4SiyRl/Ab1Vksqkgv7hwR1c7J4+7+Nt3rDd16Fr2XToT87t0JlfOn6vszaPj10qvU7836U+/rx9cs1w53UPEdfF+AmDChhdW2TymZf+aS2QfnckyxdXKHjXUhdDw3f09BegsNdnVxXnvGkp0jhg9N7fjJa39k+8";
    // this is our vuforia license key --> you can get this off of the vuforia website

    private VuforiaLocalizer vuforia;
    // this will later allow you to initialize vuforia. THIS is a particular instance of our vuforia engine

    private TFObjectDetector tfod;
    // this will later allow you to use TensorFlow.  This is a particular instance of the TensorFlow engine.

    /*
    Vuforia will feed its information and pictures it finds into TensorFlow for further analysis!
     */

    @Override
    public void runOpMode() {
        // this is what happens in the autonomous code.

        initVuforia(); // initialize vuforia first
        initTFOD(); // then initialize tensor flow.  This is because vuforia is used to feed the images into tensor flow, meaning it needs to be connected first

        if (tfod != null) {
            // aka the tensor flow has been initialized successfully.

            tfod.activate();
            // turn the tensorflow on so it starts reading.

            tfod.setZoom(2.5, 16.0/9.0);
            // zooms into what tensor flow is seeing to mimic zooming with camera.  Makes everything more readable.
        }

        waitForStart(); // keeps robot still until the play button is pressed after init.

        if (opModeIsActive()) {
            // if the play button has been pressed.

            while (opModeIsActive()) {
                // while we are still running (time hasn't run out!)

                if (tfod != null) {
                    // tensor flow is still running.

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    // curates a list of things that the camera recognized.  When it does not see anything new, it becomes NULL.
                    // updatedRecongitions holds what was found.

                    if (updatedRecognitions != null) {
                        // something is found

                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // says how many is found.

                        int i = 0; // this is a counter for the loop.
                        for (Recognition recognition : updatedRecognitions) {
                            // for each recogniton in updated recognitions (that's what the colon means!  You learn something new everyday :D)

                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            // gets what the recognized object is.

                            telemetry.addData(String.format("  left,top (%d)", i),"%.03f", "%.03f", recognition.getLeft(), recognition.getTop());
                            // get what's in the left and top.

                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f", "%.03f", recognition.getRight(), recognition.getBottom());
                            // get what's in the right and bottom.  I think this might say where in the screen it is.

                            // also, all of the percentages and stuff is a way of organizing the words in the telemetry kind of like a table.  Look up Java Formatting for information on that.
                            i++; // update counter.
                        }

                        telemetry.update();
                        // update telemetry.(aka update what it says in the console on phone!)



                    }
                }
            }
        }
    }
