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

    private static final String[] LABELS = {
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

            tfod.setZoom(1.0, 16.0/9.0);
            // magnification must be at least 1.0
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

                            telemetry.addData("Object", recognition.getLabel());
                            // gets what the recognized object is.

                            telemetry.addData("left", recognition.getLeft());
                            // get what's in the left

                            telemetry.addData("top", recognition.getTop());
                            // get what's in the top

                            telemetry.addData("right", recognition.getRight());
                            // get what's in the right

                            telemetry.addData("bottom", recognition.getBottom());
                            // get what's in the bottom

                            sleep(500);
                        }

                        telemetry.update();
                        // update telemetry.(aka update what it says in the console on phone!)
                    }
                }
            }
        }
    }

    private void initVuforia() {
        // this will initialize vuforia.

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        // creates a parameter object to collect necessary paramters and set to our vuforia localizer.

        parameters.vuforiaLicenseKey = VUFORIA_KEY; // sets the vuforia key, which gives us access

        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // sets up a camera that will be used with this program

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // makes vuforia object with said paramters
    }

    private void initTFOD() {
        // this will initialize tensor flow lite

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId","id",hardwareMap.appContext.getPackageName());
        // completely honest, not sure what this does, but I think it gets everything from the SDK to run this stuff

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        // creates the parameters with the default settings

        tfodParameters.minResultConfidence = 0.7f; // this is how sure the computer has to be to say somethhing is what it is
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // creates the tensor flow, but also links it with vuforia

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS); // loads the objects that can be detected.
    }
}
