package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Autonomous Path Maker Mark II", group="Tools")
@Disabled
public class Autonomous_Path_Maker_Mark_II extends LinearOpMode
{
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    //This is a ratio for ratio things. About 2000 Encoder Ticks to a 90 Degree Turn. Default is 22.22, Adjust to deal with drift if needed.
    static final double rotToEncoder = 2065/90;

    //This is the onboard gyroscope, pretty neat.
//    BNO055IMU imu;
//    Orientation angles;
    int currentAngle = 0;
    double currentX = 0;
    double currentY = 0;


    @Override
    public void runOpMode()  {

        //This sets up the gryoscope for use.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);

        waitForStart();

        leftDrive  = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");

        while(opModeIsActive()) {

            //Put Movement Here
            rotateToAngle(90);
            rotateToAngle(0);
            moveToTarget(-0.75, 1, 0.5);

            moveToOrigin(0.5);
            rotateToAngle(0);

            telemetry.addData("Current Rotation", currentAngle);
            telemetry.update();

            sleep(5000);
//            moveToTarget(0, 0.1, 0.5);
//            moveToTarget(0, 0.1, -0.5);

            stop();
        }

          }

    /* Valued Information:
    Encoder Value For One Rotation: 537.6
    Wheel Circumference: 0.32 Meters
    1 Meter In Encoder Values: 1620
     */

    //This smooth jazz is written like someone transposed a screeching cat, but at least it runs.
    public void moveToTarget(double targetX, double targetY, double desiredSpeed){

        //Sets up a boolean for if the rotation section goes left or right.
        boolean goRight = true;

        //This chunk of code takes the target X and Y values, gives us a hypotenuse and angle.
        double hypotenuse = Math.sqrt(targetX * targetX + targetY * targetY);
        double angleTan = (targetY/targetX);
        double angleMath = Math.atan(angleTan) * 180/3.14;
        double angle = 90 - angleMath;

        if(targetY < 0){
            angle += 180;
        }

        //Outlying Conditions For Math Or Something, I don't know, I just want working code.
        if(targetX == 0){
            if(targetY != 0){
                if(targetY > 0){
                    angle = 0;
                }
                if(targetY < 0){
                    angle = 180;
                }
            }
        }
        if(targetY == 0){
            if(targetX != 0){
                if(targetX > 0){
                    angle = 90;
                }
                if(targetX < 0){
                    angle = -90;
                }
            }
        }

        //Logic For Rotation.
        if(angle > 0){
            goRight = true;
        }
        if(angle < 0){
            goRight = false;
        }
        if(angle == 0){
            if(currentAngle > 0){
                goRight = false;
            }
            if(currentAngle < 0){
                goRight = true;
            }
        }

        telemetry.addData("Current Rotation", currentAngle);
        telemetry.addData("Current X", currentX);
        telemetry.addData("Current Y", currentY);
        telemetry.update();

        //Encoder Stuff For Encoder People.
        int encoderRotation = (int) Math.round(angle * rotToEncoder);

        //Sees if we gave it a negative speed, allows us to go backwards.
        boolean goBack = false;
        if(desiredSpeed < 0){
            goBack = true;
            desiredSpeed = Math.abs(desiredSpeed);
        }



        boolean complete = false;

        //This is to check for dividing by 0 once more, it'll set the hypotenuse to whatever the target Y value is. Since we only divide by 0 for vertical lines, whatever the target Y value is will be the hypotenuse.
        if(hypotenuse != hypotenuse){
            hypotenuse = Math.abs(targetY);
        }

        //Multiplies the hypotenuse value by the length of one meter in encoder values.
        int encoderDistance = (int) Math.round(hypotenuse * 1620);

        boolean targetLocked = false;

        //Makes sure the program doesn't skip to the next part without completing the encoder stuff.
        while(!complete && opModeIsActive()) {

            //This updates the gyroscope, and lets us see the current angle.
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            int distanceToturn = Math.abs(encoderRotation - currentAngle);

            //This code simply turns the robot until it reaches a desired angle, by comparing current angle to the one we want.
            if (!targetLocked) {
                if (goRight){
                    wheelsTurnRight();
                    encoderDrive(distanceToturn, 0.5);
                    targetLocked = true;
                }
                if (!goRight){
                    wheelsTurnLeft();
                    encoderDrive(distanceToturn, 0.5);
                    targetLocked = true;
                }

            }

            //Once at the target angle, the robot will switch to encoders and travel the distance of the hypotenuse.
            if (targetLocked) {

                //Logic for going forwards or backwards.
                if(goBack){
                    wheelsBackwards();
                }
                if(!goBack){
                    wheelsForward();
                }

                encoderDrive(encoderDistance, desiredSpeed);

                //Basically Odometry Stuff.
                currentAngle = encoderRotation;
                currentX = currentX + targetX;
                currentY = currentY + targetY;

                //Breaks the loop by setting complete to true.
                complete = true;
                telemetry.update();
            }
            telemetry.addData("Current Rotation", currentAngle);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.update();
        }
//        sleep(5000);
    }

    //This is just the previous part of the code but with just the rotation parts. Allows rotation without movement, simply input an angle in degrees. Radians are for sinners.
    public void rotateToAngle(double angle){

        boolean isDone = false;
        boolean goRight = true;
        int encoderRotation = (int) Math.round(angle*rotToEncoder);
        angle = angle - 90;
        int distanceToturn = Math.abs(encoderRotation - currentAngle);
        if(angle > 0){
            goRight = true;
        }
        if(angle < 0){
            goRight = false;
        }
        while(!isDone && opModeIsActive()){
            if (goRight){
                wheelsTurnRight();
                encoderDrive(distanceToturn, 0.5);
                isDone = true;
            }
            if (!goRight){
                wheelsTurnLeft();
                encoderDrive(distanceToturn, 0.5);
                isDone = true;
            }

        }

        currentAngle = encoderRotation;

    }

    //Standard encoder stuff. Nothing cool here.
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

    //Direction stuff for directional things.
    public void wheelsForward(){
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void wheelsBackwards(){
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    //Wooooah it goes sideways. We really are in the future.
    public void wheelsSideways(){
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    public void wheelsTurn(){
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void wheelsTurnRight(){
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void wheelsTurnLeft(){
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        BleftDrive.setDirection(DcMotor.Direction.REVERSE);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    //Weeeeeeeeee.
    public void wheelsSpin(double speed){
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        BleftDrive.setPower(speed);
        BrightDrive.setPower(speed);
    }

    //Stop wheels function makes the wheels stop. What more could you ask for?
    public void wheelsStop(){

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        BleftDrive.setPower(0);
        BrightDrive.setPower(0);
    }

    public void moveToOrigin(double desiredSpeed){

        //Sets up a boolean for if the rotation section goes left or right.
        boolean goRight = true;

        //Odometry Stuff Once More, this chunk allows the robot to find a coordinate on the field and take the shortest path there.
        double targetX = currentX;
        double targetY =  currentY;

        //This chunk of code takes the target X and Y values, gives us a hypotenuse and angle.
        double hypotenuse = Math.sqrt(targetX * targetX + targetY * targetY);
        double angleTan = (targetY/targetX);
        double angleMath = Math.atan(angleTan) * 180/3.14;
        double angle = 90 - angleMath;

        //Outlying Conditions For Math Or Something, I don't know, I just want working code.
        if(targetX == 0){
            if(targetY != 0){
                if(targetY > 0){
                    angle = 0;
                }
                if(targetY < 0){
                    angle = 180;
                }
            }
        }

        if(targetY == 0){
            if(targetX != 0){
                if(targetX > 0){
                    angle = 90;
                }
                if(targetX < 0){
                    angle = -90;
                }
            }
        }

        //Logic For Rotation.
        if(angle > 0){
            goRight = true;
        }
        if(angle < 0){
            goRight = false;
        }
        if(angle == 0){
            if(currentAngle > 0){
                goRight = false;
            }
            if(currentAngle < 0){
                goRight = true;
            }
        }

        if(targetX < 0){
            if(targetY < 0){
                angle -=180;
            }
        }

        if(currentX < 0){
            angle *= -1;
        }

        telemetry.addData("Angle", angle);
        telemetry.update();

        //Encoder Stuff For Encoder People.
        int encoderRotation = (int) Math.round(angle * rotToEncoder);

        //Sees if we gave it a negative speed, allows us to go backwards.
        boolean goBack = false;
        if(desiredSpeed > 0){
            goBack = true;
            desiredSpeed = Math.abs(desiredSpeed);
        }



        boolean complete = false;

        //This is to check for dividing by 0 once more, it'll set the hypotenuse to whatever the target Y value is. Since we only divide by 0 for vertical lines, whatever the target Y value is will be the hypotenuse.
        if(hypotenuse != hypotenuse){
            hypotenuse = Math.abs(targetY);
        }

        //Multiplies the hypotenuse value by the length of one meter in encoder values.
        int encoderDistance = (int) Math.round(hypotenuse * 1620);

        boolean targetLocked = false;

        //Makes sure the program doesn't skip to the next part without completing the encoder stuff.
        while(!complete && opModeIsActive()) {

            //This updates the gyroscope, and lets us see the current angle.
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            int distanceToturn = Math.abs(encoderRotation - currentAngle);

            //This code simply turns the robot until it reaches a desired angle, by comparing current angle to the one we want.
            if (!targetLocked) {
                if (goRight){
                    wheelsTurnRight();
                    encoderDrive(distanceToturn, 0.5);
                    targetLocked = true;
                }
                if (!goRight){
                    wheelsTurnLeft();
                    encoderDrive(distanceToturn, 0.5);
                    targetLocked = true;
                }

            }

            //Once at the target angle, the robot will switch to encoders and travel the distance of the hypotenuse.
            if (targetLocked) {

                //Logic for going forwards or backwards.
                if(goBack){
                    wheelsBackwards();
                }
                if(!goBack){
                    wheelsForward();
                }

                encoderDrive(encoderDistance, desiredSpeed);

                //Basically Odometry Stuff.
//                if(currentX > 0){
//                    currentAngle = -encoderRotation;
//                }
//                if(currentX < 0){
//                    currentAngle = encoderRotation;
//                }
                if(currentX >= 0){
                    currentAngle = (int) Math.round(angle);
                    currentAngle *= rotToEncoder;
                    telemetry.addData("Test", "Posi X");
                    telemetry.update();
                }
                if(currentX < 0){
                    currentAngle = (int) Math.round(angle) ;
                    currentAngle *= rotToEncoder;
                    telemetry.addData("Test", "Negi X");
                    telemetry.addData("Current Rotation", currentAngle);
                    telemetry.update();
                }

                currentX = 0;
                currentY = 0;

                //Breaks the loop by setting complete to true.
                complete = true;
                telemetry.update();
            }
            telemetry.addData("Hypotenuse", hypotenuse);
//            telemetry.addData("Angle", angle);
//            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Distance Needed", encoderDistance);
            telemetry.addData("Target Locked?", targetLocked);
            telemetry.update();
        }
    }

    public void moveToCoordinates(double desiredX, double desiredY, double desiredSpeed){

        //Sets up a boolean for if the rotation section goes left or right.
        boolean goRight = true;

        //Odometry Stuff Once More, this chunk allows the robot to find a coordinate on the field and take the shortest path there.
        double targetX = desiredX - currentX;
        double targetY =  desiredY - currentY;

        //This chunk of code takes the target X and Y values, gives us a hypotenuse and angle.
        double hypotenuse = Math.sqrt(targetX * targetX + targetY * targetY);
        double angleTan = (targetY/targetX);
        double angleMath = Math.atan(angleTan) * 180/3.14;
        double angle = 90 - angleMath;


//        if(enterY < 0){
//            angle += 180;
//        }

        //Outlying Conditions For Math Or Something, I don't know, I just want working code.
        if(targetX == 0){
            if(targetY != 0){
                if(targetY > 0){
                    angle = 0;
                }
                if(targetY < 0){
                    angle = 180;
                }
            }
        }
        if(targetY == 0){
            if(targetX != 0){
                if(targetX > 0){
                    angle = 90;
                }
                if(targetX < 0){
                    angle = -90;
                }
            }
        }

        //Logic For Rotation.
        if(angle > 0){
            goRight = true;
        }
        if(angle < 0){
            goRight = false;
        }
        if(angle == 0){
            if(currentAngle > 0){
                goRight = false;
            }
            if(currentAngle < 0){
                goRight = true;
            }
        }

        if(targetX < 0){
            if(targetY < 0){
                angle -=180;
            }
        }

        telemetry.addData("Angle", angle);
        telemetry.update();

        //Encoder Stuff For Encoder People.
        int encoderRotation = (int) Math.round(angle * rotToEncoder);

        //Sees if we gave it a negative speed, allows us to go backwards.
        boolean goBack = false;
        if(desiredSpeed > 0){
            goBack = false;
            desiredSpeed = Math.abs(desiredSpeed);
        }



        boolean complete = false;

        //This is to check for dividing by 0 once more, it'll set the hypotenuse to whatever the target Y value is. Since we only divide by 0 for vertical lines, whatever the target Y value is will be the hypotenuse.
        if(hypotenuse != hypotenuse){
            hypotenuse = Math.abs(targetY);
        }

        //Multiplies the hypotenuse value by the length of one meter in encoder values.
        int encoderDistance = (int) Math.round(hypotenuse * 1620);

        boolean targetLocked = false;

        //Makes sure the program doesn't skip to the next part without completing the encoder stuff.
        while(!complete && opModeIsActive()) {

            //This updates the gyroscope, and lets us see the current angle.
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            int distanceToturn = Math.abs(encoderRotation - currentAngle);

            //This code simply turns the robot until it reaches a desired angle, by comparing current angle to the one we want.
            if (!targetLocked) {
                if (goRight){
                    wheelsTurnRight();
                    encoderDrive(distanceToturn, 0.5);
                    targetLocked = true;
                }
                if (!goRight){
                    wheelsTurnLeft();
                    encoderDrive(distanceToturn, 0.5);
                    targetLocked = true;
                }

            }

            //Once at the target angle, the robot will switch to encoders and travel the distance of the hypotenuse.
            if (targetLocked) {

                //Logic for going forwards or backwards.
                if(goBack){
                    wheelsBackwards();
                }
                if(!goBack){
                    wheelsForward();
                }

                encoderDrive(encoderDistance, desiredSpeed);

                //Basically Odometry Stuff.


                currentX = desiredX;
                currentY = desiredY;

                //Breaks the loop by setting complete to true.
                complete = true;
                telemetry.update();
            }
            telemetry.addData("Hypotenuse", hypotenuse);
            telemetry.addData("Angle", angle);
//            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Distance Needed", encoderDistance);
            telemetry.addData("Target Locked?", targetLocked);
            telemetry.update();
        }
    }

}
