package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Rotation With Encoders", group="Test")
public class RotationWithEncoders extends LinearOpMode
{
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    BNO055IMU imu;
    Orientation angles;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode()  {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        leftDrive  = hardwareMap.get(DcMotor.class, "FL");
        rightDrive = hardwareMap.get(DcMotor.class, "FR");
        BleftDrive  = hardwareMap.get(DcMotor.class, "BL");
        BrightDrive = hardwareMap.get(DcMotor.class, "BR");

        telemetry.addData("Status", "Initialized");

        while(opModeIsActive()) {


            moveToTarget(-0.5, 0.5, 0.5);
            moveToTarget(-0.5, 0.5, -0.5);
            moveToTarget(0.5, 0.5, 0.5);
            moveToTarget(0.5, 0.5, -0.5);

            stop();

        }
          }



    /* Valued Information:
    Encoder Value For One Rotation: 537.6
    Wheel Circumference: 0.32 Meters
    1 Meter In Encoder Values: 1620
     */

    public void moveToTarget(double targetX, double targetY, double desiredSpeed){

        double hypotenuse = Math.sqrt(targetX * targetX + targetY * targetY);
        double angleTan = (targetY/targetX);
        double angleMath = Math.atan(angleTan) * 180/3.14;
        double angle = angleMath - (90 * targetX/Math.abs(targetX));
        double persistantRotation = 0;


        int angleEncoder = (int) Math.round(angle*22.2222);


        boolean goBack = false;
        if(desiredSpeed < 0){
            goBack = true;
            desiredSpeed = Math.abs(desiredSpeed);
        }

        if(angle != angle){
            if(targetY > 0){
                angle = 0;
            }
            if(targetY < 0){
                angle = 180;
            }
        }

        boolean complete = false;

        if(hypotenuse != hypotenuse){
            hypotenuse = Math.abs(targetY);
        }

        int encoderDistance = (int) Math.round(hypotenuse * 1620);

        boolean targetLocked = false;

        while(!complete) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);



            if (!targetLocked) {
                if (angles.firstAngle > angle) {
                    //angleEncoder = Math.round(angleEncoder-angles.firstAngle);
                    //rotationEncoder(angleEncoder, 0.05);
                    telemetry.update();
                }

                if (angles.firstAngle < angle) {
                    //angleEncoder = Math.round(angleEncoder-angles.firstAngle);
                   //rotationEncoder(angleEncoder, -0.05);
                    telemetry.update();
                }

                if (angles.firstAngle < angle + 1 && angles.firstAngle > angle - 1) {
                    targetLocked = true;
                }
            }

            if (targetLocked) {
                if(goBack){
                    wheelsBackwards();
                }
                if(!goBack){
                    wheelsForward();
                }
                encoderDrive(encoderDistance, desiredSpeed);
                complete = true;
                telemetry.addData("Complete", "Complete");
                telemetry.update();
            }

            telemetry.addData("encoderAngles", angleEncoder);
            //telemetry.addData("Hypotenuse", hypotenuse);
            telemetry.addData("Angle", angle);
            telemetry.addData("Heading", angles.firstAngle);
            //telemetry.addData("Distance Needed", encoderDistance);
            telemetry.addData("Target Locked?", targetLocked);
            telemetry.update();
        }
        persistantRotation = persistantRotation + angle;
        telemetry.addData("Persistant Rotation", persistantRotation);
    }

    public void rotateToAngleEncoders(double angle){

        boolean isDone = false;

        double desiredAngle = angle;

        while(!isDone){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


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

    public void wheelsSideways(){
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);
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
    public void rotationEncoder(int inputAngle, double desiredSpeed){

        int desiredEncoder = (int) Math.round(inputAngle * 22.22222);

        boolean goRight = false;
        if (desiredSpeed > 0){
            goRight = true;
        }
        if(desiredSpeed < 0){
            goRight = false;
        }

        if(goRight){
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            BleftDrive.setDirection(DcMotor.Direction.FORWARD);
            BrightDrive.setDirection(DcMotor.Direction.FORWARD);
        }

        if (!goRight){
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            BleftDrive.setDirection(DcMotor.Direction.REVERSE);
            BrightDrive.setDirection(DcMotor.Direction.REVERSE);
        }

        desiredSpeed = Math.abs(desiredSpeed);

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
            telemetry.addData("Encoder Count", leftDrive.getCurrentPosition());
            telemetry.addData("Target Encoder", desiredEncoder);
            telemetry.update();
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
}
