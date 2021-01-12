package org.firstinspires.ftc.teamcode.oldtests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp(name="Gamepad Test", group="Basic")
public class Gamepad_Test extends LinearOpMode{
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    boolean doMove = true;

    double leftTurn = 1;
    double rightTurn = 1;
    double adjustTurn = 0;

    double currentAngle = 0;

    double currentTime = -1;

    double encoderStep = -1;

    double sinY = 0;

    double derivOfFunct = 0;

    //Line Shit
    double a = 2;
    double b = Math.PI;
    double c = 1;
    double slopeToAngle = 0;




    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {

        //This sets up the gryoscope for use.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        double startTime = System.currentTimeMillis();

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

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);

        while(opModeIsActive()){

            currentTime = ((System.currentTimeMillis() - startTime)/10000*6) - (1/c);

            if(currentTime > 0){
                leftDrive.setPower(0);
                BleftDrive.setPower(0);
                rightDrive.setPower(0);
                BrightDrive.setPower(0);
                stop();
            }

            sinY = (a*Math.asin(currentTime))/b;

            derivOfFunct = a/(b*Math.sqrt(1-(currentTime*currentTime)));

            slopeToAngle = 90 - (Math.atan(derivOfFunct)*180/Math.PI);

            //Gyro Stuff
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = Math.round(-angles.firstAngle);

            if(currentAngle < slopeToAngle){
                rightTurn = 0;
                if(currentAngle > (slopeToAngle - 10)){
                    rightTurn = 1 - Math.abs((currentAngle - slopeToAngle)/5);
                }
            }
            else rightTurn = 1;
            if(currentAngle > slopeToAngle){
                leftTurn = 0;
                if(currentAngle < (slopeToAngle + 10)){
                    leftTurn = 1 - Math.abs((currentAngle - slopeToAngle)/5);
                }
            }
            else leftTurn = 1;

            leftDrive.setPower(1*leftTurn);
            BleftDrive.setPower(1*leftTurn);
            rightDrive.setPower(1*rightTurn);
            BrightDrive.setPower(1*rightTurn);



//            telemetry.addData("Can Move?", doMove);
//            telemetry.addData("Current Angle?", currentAngle);
//            telemetry.addData("Turn Adjust", adjustTurn);
//            telemetry.addData("Left Y", gamepad1.left_stick_y);
//            telemetry.addData("Left Encoder", leftDrive.getCurrentPosition());
//            telemetry.addData("Right Encoder", rightDrive.getCurrentPosition());
//            telemetry.addData("Time", currentTime);
            telemetry.addData("Left Turn", leftTurn);
            telemetry.addData("Right Turn", rightTurn);
            telemetry.addData("Sin X", currentTime);
            telemetry.addData("Sin Y", sinY);
            telemetry.addData("Derivative", derivOfFunct);
            telemetry.addData("Angle Of Line", slopeToAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.update();
        }


    }
}


