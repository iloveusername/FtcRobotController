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

package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Concept: TensorFlow Object Detection Webcam", group = "Concept")
public class WebcamTest2021FEB extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

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
            "AYxODmX/////AAABmYWAatyEKEgPmDxDlll3FNFpLYOsxjH7uWQ5Hrpubiy7uQ7tcUPz5E7v5T02dtOhSTUNeFyoa0PPuVqmEdOmz6oxxlXhoL4vNjc+m965JC7kSsQw73WWpmt27bS/OLLu+hzfczq1Fa37MYvtHPn7RDUellpI6areenBG34mvIRhwHPbFhCSpxGqDLXb7zeq9v68yX0aNluJfeA67lnoCIInHyeWuTSNMrISbnEkOnzSzIkE+IcwHRvOEfn68DQMx8i39CUD0jr95XfHwFsHVAzup4pR7I66b9aPo2P/UvYEOvF6/RUZSMiExnAmXCj6suHQwPkR2WEWl6t2Ns0anJOQ0dbhd6E+fnQYEq6HXc0Db";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    int i = 0;

    //This is the onboard gyroscope, pretty neat.
    BNO055IMU imu;
    Orientation angles;

    //This is a ratio for ratio things. About 2000 Encoder Ticks to a 90 Degree Turn. Default is ~22, Adjust to deal with encoder loss if needed. 1620 ticks for one meter, I think. I don't have a meter stick, so who really knows.
    static final double rotToEncoder = 2065 / 90;
    static final double meterToEncoder = 1620;

    //Sets up odometry.
    double currentAngle = 0;
    double currentX = 0;
    double currentY = 0;
    boolean trackEncoders = false;
    boolean doneTurn = false;
    double averagePos;
    String currentDirection = "Up";
    String dirCheck = currentDirection;

    //Initial Checkpoint is the origin.
    double checkX = 0;
    double checkY = 0;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");

        telemetry.update();//This sets up the gryoscope for use.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Lines 62-82 of code set up the motors for use.
        leftDrive = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        wheelDirection("up");

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      // step through the list of recognitions and display boundary info.

                      for (Recognition recognition : updatedRecognitions) {
//                          telemetry.addData("Label", recognition.getLabel());
                          if(recognition.getLabel() == "Single"){

                          }
                          if(recognition.getLabel() == "Quad"){

                          }
                          if(recognition.getLabel() != "Single" && recognition.getLabel() != "Quad"){

                          }
                      }
                      telemetry.addData("I", i);
                      telemetry.update();
                    }

                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
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
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void goToTarget(double targetX, double targetY, double targetSpeed){
        //Sets up a switch to toggle this function on and off with.
        boolean isDone = false;

        //With Absolute Values, We Can Find The Angles We Want Without Crying. No promises.
        double adjustedX = Math.abs(targetX);
        double adjustedY = Math.abs(targetY);
        int Quadrant = 1;

        //This Determines The Quadrant Of The Angle.
        if(targetX > 0){
            if(targetY > 0){
                Quadrant = 1;
            }
            if(targetY < 0){
                Quadrant = 4;
            }
        }
        if(targetX < 0){
            if(targetY > 0){
                Quadrant = 2;
            }
            if(targetY < 0){
                Quadrant = 3;
            }
        }

        //Finds Hypotenuse and the Theta value of the triangle.
        double HypotenuseOfTri = Math.sqrt((targetX * targetX) + (targetY * targetY));
        double AngleOfTri = Math.atan(adjustedY/adjustedX) * 180/Math.PI;

        //How much we need to turn depends on the quadrant we are currently in.
        switch (Quadrant){
            case 1:
                AngleOfTri = 90 - AngleOfTri;
                break;
            case 2:
                AngleOfTri = -90 + AngleOfTri;
                break;
            case 3:
                AngleOfTri = -90 - AngleOfTri;
                break;
            case 4:
                AngleOfTri = 90 + AngleOfTri;
                break;
        }

        //This determines if we are going vertical or horizontal, and sets the angle to whatever it needs to be.
        if(targetX == 0){
            if(targetY != 0){
                HypotenuseOfTri = Math.abs(targetY);
                if(targetY > 0){
                    AngleOfTri = 0;
                }
                if(targetY < 0){
                    AngleOfTri = -180;
                }
            }
        }
        if(targetY == 0){
            if(targetX != 0){
                HypotenuseOfTri = Math.abs(targetX);
                if(targetX > 0){
                    AngleOfTri = 90;
                }
                if(targetX < 0){
                    AngleOfTri = -90;
                }
            }
        }

        //Determine turning direction.
        if(AngleOfTri > currentAngle){
            wheelDirection("turnRight");
        }
        if(AngleOfTri < currentAngle){
            wheelDirection("turnLeft");
        }

        while(!isDone && opModeIsActive()){

            //Refresh the gyroscope.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = -angles.firstAngle;

            rotateToAngle(AngleOfTri);

            //Moves the robot forward for the distance of the hypotenuse.
            wheelDirection("up");
            encoderDrive((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed);

            //Telemetry stuff for debugging.
            telemetry.addData("Distance To Rotate", (AngleOfTri - currentAngle));
            telemetry.addData("Angle Of Attack", AngleOfTri);
            telemetry.addData("Hypotenuse", HypotenuseOfTri);
            telemetry.addData("Quadrant", Quadrant);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Current Rot", currentAngle);
            telemetry.update();

            //Updates current position and rotation.
            currentX += targetX;
            currentY += targetY;

            isDone = true;

        }


    }

    public void goToCoordinates(double coordX, double coordY, double targetSpeed){
        //Sets up a switch to toggle this function on and off with.
        boolean isDone = false;

        //Take our wanted coordinates, subtract where we are current at, and we should go to where we wanna go. Hopefully.
        double targetX = coordX - currentX;
        double targetY = coordY - currentY;

        //With Absolute Values, We Can Find The Angles We Want Without Crying. No promises.
        double adjustedX = Math.abs(targetX);
        double adjustedY = Math.abs(targetY);
        int Quadrant = 1;

        //This Determines The Quadrant Of The Angle.
        if(targetX > 0){
            if(targetY > 0){
                Quadrant = 1;
            }
            if(targetY < 0){
                Quadrant = 4;
            }
        }
        if(targetX < 0){
            if(targetY > 0){
                Quadrant = 2;
            }
            if(targetY < 0){
                Quadrant = 3;
            }
        }

        //Finds Hypotenuse and the Theta value of the triangle.
        double HypotenuseOfTri = Math.sqrt((targetX * targetX) + (targetY * targetY));
        double AngleOfTri = Math.atan(adjustedY/adjustedX) * 180/Math.PI;

        //How much we need to turn depends on the quadrant we are currently in.
        switch (Quadrant){
            case 1:
                AngleOfTri = 90 - AngleOfTri;
                break;
            case 2:
                AngleOfTri = -90 + AngleOfTri;
                break;
            case 3:
                AngleOfTri = -90 - AngleOfTri;
                break;
            case 4:
                AngleOfTri = 90 + AngleOfTri;
                break;
        }

        //This determines if we are going vertical or horizontal, and sets the angle to whatever it needs to be.
        if(targetX == 0){
            if(targetY != 0){
                HypotenuseOfTri = Math.abs(targetY);
                if(targetY > 0){
                    AngleOfTri = 0;
                }
                if(targetY < 0){
                    AngleOfTri = -180;
                }
            }
        }
        if(targetY == 0){
            if(targetX != 0){
                HypotenuseOfTri = Math.abs(targetX);
                if(targetX > 0){
                    AngleOfTri = 90;
                }
                if(targetX < 0){
                    AngleOfTri = -90;
                }
            }
        }

        //Determine turning direction.
        if(AngleOfTri > currentAngle){
            wheelDirection("turnRight");
        }
        if(AngleOfTri < currentAngle){
            wheelDirection("turnLeft");
        }

        //Refresh the gyroscope.
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentAngle = -angles.firstAngle;

        //Turns the robot using encoders for accuracy.
        rotateToAngle(AngleOfTri);

        //Moves the robot forward for the distance of the hypotenuse.
        wheelDirection("up");
        encoderDrive((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed);

        //Updates current position and rotation.
        currentAngle = AngleOfTri;
        currentX = coordX;
        currentY = coordY;

        //Telemetry stuff for debugging.
        telemetry.addData("Angle Of Attack", AngleOfTri);
        telemetry.addData("Hypotenuse", HypotenuseOfTri);
        telemetry.addData("Quadrant", Quadrant);
        telemetry.addData("Current X", currentX);
        telemetry.addData("Current Y", currentY);
        telemetry.addData("Current Rot", currentAngle);
        telemetry.update();

    }

    public void goToOrigin(double targetSpeed){
        //Sets up a switch to toggle this function on and off with.
        boolean isDone = false;

        //If we take our current position, then input the X and Y values into the trig stuff, then just go backwards instead of forwards, we go straight back to origin. Thanks, geometry.
        double targetX = currentX;
        double targetY = currentY;

        //With Absolute Values, We Can Find The Angles We Want Without Crying. No promises.
        double adjustedX = Math.abs(targetX);
        double adjustedY = Math.abs(targetY);
        int Quadrant = 1;

        //This Determines The Quadrant Of The Angle.
        if(targetX > 0){
            if(targetY > 0){
                Quadrant = 1;
            }
            if(targetY < 0){
                Quadrant = 4;
            }
        }
        if(targetX < 0){
            if(targetY > 0){
                Quadrant = 2;
            }
            if(targetY < 0){
                Quadrant = 3;
            }
        }

        //Finds Hypotenuse and the Theta value of the triangle.
        double HypotenuseOfTri = Math.sqrt((targetX * targetX) + (targetY * targetY));
        double AngleOfTri = Math.atan(adjustedY/adjustedX) * 180/Math.PI;

        //How much we need to turn depends on the quadrant we are currently in.
        switch (Quadrant){
            case 1:
                AngleOfTri = 90 - AngleOfTri;
                break;
            case 2:
                AngleOfTri = -90 + AngleOfTri;
                break;
            case 3:
                AngleOfTri = -90 - AngleOfTri;
                break;
            case 4:
                AngleOfTri = 90 + AngleOfTri;
                break;
        }

        //This determines if we are going vertical or horizontal, and sets the angle to whatever it needs to be.
        if(targetX == 0){
            if(targetY != 0){
                HypotenuseOfTri = Math.abs(targetY);
                if(targetY > 0){
                    AngleOfTri = 0;
                }
                if(targetY < 0){
                    AngleOfTri = -180;
                }
            }
        }
        if(targetY == 0){
            if(targetX != 0){
                HypotenuseOfTri = Math.abs(targetX);
                if(targetX > 0){
                    AngleOfTri = 90;
                }
                if(targetX < 0){
                    AngleOfTri = -90;
                }
            }
        }

        //Determine turning direction.
        if(AngleOfTri > currentAngle){
            wheelDirection("turnRight");
        }
        if(AngleOfTri < currentAngle){
            wheelDirection("turnLeft");
        }

        while(!isDone && opModeIsActive()){
            //Refresh the gyroscope.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = -angles.firstAngle;

            rotateToAngle(AngleOfTri);

            //Moves the robot forward for the distance of the hypotenuse.
            wheelDirection("down");
            encoderDrive((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed);

            //Telemetry stuff for debugging.
            telemetry.addData("Distance To Rotate", (AngleOfTri - currentAngle));
            telemetry.addData("Angle Of Attack", AngleOfTri);
            telemetry.addData("Hypotenuse", HypotenuseOfTri);
            telemetry.addData("Quadrant", Quadrant);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Current Rot", currentAngle);
            telemetry.update();

            //Updates current position and rotation.
            currentAngle = AngleOfTri;
            currentX = 0;
            currentY = 0;

            isDone = true;
        }
    }

//    public void rotateToAngleOld(double angle){
//        boolean isDone = false;
//
//        while(!isDone) {
//            //Determine if we need to spin left or right.
//            if (angle > currentAngle) {
//                wheelDirection("turnRight");
//            }
//            if (angle < currentAngle) {
//                wheelDirection("turnLeft");
//            }
//
//            //Turns the robot using encoders for accuracy. Adjust speed if you want.
//            int encoderTurn = (int) Math.round(((angle - currentAngle) * rotToEncoder));
//            encoderTurn = Math.abs(encoderTurn);
//            encoderDrive(encoderTurn, 0.75);
//
//            isDone = true;
//        }
//    }

    //Compares your current angle to your desired angle, and finds the shortest angle to turn to reach it.
    public void rotateToAngle(double angle){
        //Angle two is the sum of the distances to reach 180 from the current angle and from the desired angle.
        double angle2 = ((180-Math.abs(currentAngle)) + (180 - Math.abs(angle)));
        //Angle three is simply the difference betweeen desired angle and current angle.
        double angle3 =  angle - currentAngle;
        telemetry.addData("Angle2", angle2);
        telemetry.addData("Angle3", angle3);
        telemetry.update();
        if(angle2 < Math.abs(angle3)){
            if (angle2 > currentAngle) {
                wheelDirection("turnLeft");
            }
            if (angle2 < currentAngle) {
                wheelDirection("turnRight");
            }
            int encoderTurn = (int) Math.round(((angle2) * rotToEncoder));
            encoderTurn = Math.abs(encoderTurn);
            encoderDrive(encoderTurn, 0.75);
        }
        else if(angle2 > Math.abs(angle3)){
            if (angle > currentAngle) {
                wheelDirection("turnRight");
            }
            if (angle < currentAngle) {
                wheelDirection("turnLeft");
            }
            int encoderTurn = (int) Math.round(((angle3) * rotToEncoder));
            encoderTurn = Math.abs(encoderTurn);
            encoderDrive(encoderTurn, 0.75);
        }


    }

    public void wheelDirection(String dir){
        switch (dir){
            case "up":
                currentDirection = "Up";
                leftDrive.setDirection(DcMotor.Direction.FORWARD);
                rightDrive.setDirection(DcMotor.Direction.REVERSE);
                BleftDrive.setDirection(DcMotor.Direction.FORWARD);
                BrightDrive.setDirection(DcMotor.Direction.REVERSE);
                break;
            case "down":
                currentDirection = "Down";
                leftDrive.setDirection(DcMotor.Direction.REVERSE);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);
                BleftDrive.setDirection(DcMotor.Direction.REVERSE);
                BrightDrive.setDirection(DcMotor.Direction.FORWARD);
                break;
            case "left":
                currentDirection = "Left";
                leftDrive.setDirection(DcMotor.Direction.REVERSE);
                rightDrive.setDirection(DcMotor.Direction.REVERSE);
                BleftDrive.setDirection(DcMotor.Direction.FORWARD);
                BrightDrive.setDirection(DcMotor.Direction.FORWARD);
                break;
            case "right":
                currentDirection = "Right";
                leftDrive.setDirection(DcMotor.Direction.FORWARD);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);
                BleftDrive.setDirection(DcMotor.Direction.REVERSE);
                BrightDrive.setDirection(DcMotor.Direction.REVERSE);
                break;
            case "turnLeft":
                leftDrive.setDirection(DcMotor.Direction.REVERSE);
                rightDrive.setDirection(DcMotor.Direction.REVERSE);
                BleftDrive.setDirection(DcMotor.Direction.REVERSE);
                BrightDrive.setDirection(DcMotor.Direction.REVERSE);
                break;
            case "turnRight":
                leftDrive.setDirection(DcMotor.Direction.FORWARD);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);
                BleftDrive.setDirection(DcMotor.Direction.FORWARD);
                BrightDrive.setDirection(DcMotor.Direction.FORWARD);
                break;
        }
    }

    public void encoderDrive(int desiredEncoder, double desiredSpeed){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition(desiredEncoder);
        rightDrive.setTargetPosition(desiredEncoder);
        BleftDrive.setTargetPosition(desiredEncoder);
        BrightDrive.setTargetPosition(desiredEncoder);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(desiredSpeed);
        rightDrive.setPower(desiredSpeed);
        BleftDrive.setPower(desiredSpeed);
        BrightDrive.setPower(desiredSpeed);

        while(leftDrive.isBusy() || rightDrive.isBusy() || BleftDrive.isBusy() || BrightDrive.isBusy()){
            //If we press Y, it should abort whatever sequence it is in, and hopefully leave us with a decently accurate position.
            if(gamepad1.y){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = Math.round(-angles.firstAngle);
                currentY += (Math.cos(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                currentX += (Math.sin(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                currentY = (double) Math.round(currentY * 100) / 100;
                currentX = (double) Math.round(currentX * 100) / 100;
                resetCount();
                break;
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void encoderDriveDiagonal(int desiredEncoder, double desiredSpeed, boolean Right){
        desiredEncoder = (int) (desiredEncoder*1.4);
        desiredSpeed = desiredSpeed * 1.4;
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(Right) {
            leftDrive.setTargetPosition(desiredEncoder);
            BrightDrive.setTargetPosition(desiredEncoder);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setPower(desiredSpeed);
            rightDrive.setPower(0);
            BleftDrive.setPower(0);
            BrightDrive.setPower(desiredSpeed);
        }
        if(!Right){
            rightDrive.setTargetPosition(desiredEncoder);
            BleftDrive.setTargetPosition(desiredEncoder);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setPower(0);
            rightDrive.setPower(desiredSpeed);
            BleftDrive.setPower(desiredSpeed);
            BrightDrive.setPower(0);
        }

        while(leftDrive.isBusy() || rightDrive.isBusy() || BleftDrive.isBusy() || BrightDrive.isBusy()){
            //If we press Y, it should abort whatever sequence it is in, and hopefully leave us with a decently accurate position.
            if(gamepad1.y){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = Math.round(-angles.firstAngle);
                currentY +=  0.71*(Math.cos(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                currentX +=  0.71*(Math.sin(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                currentY = (double) Math.round(currentY * 100) / 100;
                currentX = (double) Math.round(currentX * 100) / 100;
                resetCount();
                break;
            }
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goToTargetBetterTurning(double coordX, double coordY, double targetSpeed){
        //Gaming
        double targetX = coordX - currentX;
        double targetY = coordY - currentY;

        //Sets up a switch to toggle this function on and off with.
        boolean isDone = false;

        //With Absolute Values, We Can Find The Angles We Want Without Crying. No promises.
        double adjustedX = Math.abs(targetX);
        double adjustedY = Math.abs(targetY);
        int targetLine = 0;
        int Quadrant = 1;

        //This Determines The Quadrant Of The Angle.
        if(targetX > 0){
            if(targetY > 0){
                Quadrant = 1;
            }
            if(targetY < 0){
                Quadrant = 4;
            }
        }
        if(targetX < 0){
            if(targetY > 0){
                Quadrant = 2;
            }
            if(targetY < 0){
                Quadrant = 3;
            }
        }

        //Finds Hypotenuse and the Theta value of the triangle.
        double HypotenuseOfTri = Math.sqrt((targetX * targetX) + (targetY * targetY));
        double AngleOfTri = Math.atan(adjustedY/adjustedX) * 180/Math.PI;

        //How much we need to turn depends on the quadrant we are currently in.
        switch (Quadrant){
            case 1:
                AngleOfTri = 90 - AngleOfTri;
                break;
            case 2:
                AngleOfTri = -90 + AngleOfTri;
                break;
            case 3:
                AngleOfTri = -90 - AngleOfTri;
                break;
            case 4:
                AngleOfTri = 90 + AngleOfTri;
                break;
        }

        //This determines if we are going vertical or horizontal, and sets the angle to whatever it needs to be.
        if(targetX == 0){
            if(targetY != 0){
                HypotenuseOfTri = Math.abs(targetY);
                if(targetY > 0){
                    AngleOfTri = 0;
                }
                if(targetY < 0){
                    AngleOfTri = -180;
                }
            }
        }
        if(targetY == 0){
            if(targetX != 0){
                HypotenuseOfTri = Math.abs(targetX);
                if(targetX > 0){
                    AngleOfTri = 90;
                }
                if(targetX < 0){
                    AngleOfTri = -90;
                }
            }
        }

        //Determine turning direction.
        if(AngleOfTri > currentAngle){
            wheelDirection("turnRight");
        }
        if(AngleOfTri < currentAngle){
            wheelDirection("turnLeft");
        }

        while(!isDone && opModeIsActive()){

            //Refresh the gyroscope.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = -angles.firstAngle;

            //Gamer Stuff For Gamers
            targetLine = 45 * (int) Math.round((AngleOfTri - currentAngle)/45);
//            if(targetLine > 180){
//                targetLine = 180;
//            }
//            else if(targetLine < -180){
//                targetLine = -180;
//            }

            //Gamer Switch Statement
            telemetry.addData("Type",targetLine);
            telemetry.update();

            //Refresh the gyroscope.
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = -angles.firstAngle;

            //Rotates to the target line.
            rotateToAngle(AngleOfTri-targetLine);

            if(targetLine == 225){
                targetLine = -135;
            }
            if(targetLine == -225){
                targetLine = 135;
            }
            if(targetLine == -180){
                targetLine = 180;
            }

            switch (targetLine){
                case -135:
                    //Moves the robot forward for the distance of the hypotenuse.
                    wheelDirection("down");
                    encoderDriveDiagonal((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed, true);
                    break;

                case -90:
                    //Moves the robot forward for the distance of the hypotenuse.
                    wheelDirection("left");
                    encoderDrive((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed);
                    break;

                case -45:
                    //Moves the robot forward for the distance of the hypotenuse.
                    wheelDirection("up");
                    encoderDriveDiagonal((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed, false);
                    break;

                case 0:
                    //Moves the robot forward for the distance of the hypotenuse.
                    wheelDirection("up");
                    encoderDrive((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed);
                    break;

                case 45:
                    //Moves the robot forward for the distance of the hypotenuse.
                    wheelDirection("up");
                    encoderDriveDiagonal((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed, true);
                    break;

                case 90:
                    //Moves the robot forward for the distance of the hypotenuse.
                    wheelDirection("right");
                    encoderDrive((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed);
                    break;

                case 135:
                    //Moves the robot forward for the distance of the hypotenuse.
                    wheelDirection("down");
                    encoderDriveDiagonal((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed, false);
                    break;

                case 180:
                    //Moves the robot forward for the distance of the hypotenuse.
                    wheelDirection("down");
                    encoderDrive((int) Math.round(HypotenuseOfTri * meterToEncoder), targetSpeed);
                    break;
            }

            //Telemetry stuff for debugging.
            telemetry.addData("Distance To Rotate", (AngleOfTri - currentAngle));
            telemetry.addData("Angle Of Attack", AngleOfTri);
            telemetry.addData("Hypotenuse", HypotenuseOfTri);
            telemetry.addData("Quadrant", Quadrant);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Current Rot", currentAngle);
            telemetry.update();

            //Updates current position and rotation.
            currentX += targetX;
            currentY += targetY;

            isDone = true;

        }


    }


    public void resetDrive(){
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetCount(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
