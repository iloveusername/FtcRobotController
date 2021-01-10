package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



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

    double currentTime = 0;

    double sinY = 0;

    double derivOfFunct = 0;

    //Line Shit
    double a = 1;
    double b = Math.PI;
    double c = 2;
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

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        BleftDrive.setDirection(DcMotor.Direction.FORWARD);
        BrightDrive.setDirection(DcMotor.Direction.REVERSE);

        while(opModeIsActive()){

            currentTime = ((System.currentTimeMillis() - startTime)/10000) - (a*0.5);

//            currentTime = -1;

            if(currentTime > a*0.5){
                stop();
            }

            sinY = (a*Math.asin(c*currentTime))/b;

            derivOfFunct = a/(b*Math.sqrt(1-((c*c) * (currentTime*currentTime))));

            slopeToAngle = 90 - (Math.atan(derivOfFunct)*180/Math.PI);

            //Gyro Stuff
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = Math.round(-angles.firstAngle);

            if(gamepad1.y){
                if(doMove){
                    doMove = false;
                }
                else{
                    doMove = true;
                }
            }

//            if(gamepad1.left_bumper){
//                adjustTurn -= 0.1;
//                sleep(500);
//            }
//            if(gamepad1.right_bumper){
//                adjustTurn += 0.1;
//                sleep(500);
//            }
//
//            if(gamepad1.x){
//                leftTurn = 0.5 + adjustTurn;
//            }
//            else{
//                leftTurn = 1;
//            }
//            if(gamepad1.b){
//                rightTurn = 0.5 + adjustTurn;
//            }
//            else{
//                rightTurn = 1;
//            }


            if(currentAngle < slopeToAngle) rightTurn = 0;
            else rightTurn = 1;
            if(currentAngle > slopeToAngle) leftTurn = 0;
            else leftTurn = 1;


            if(doMove){
                leftDrive.setPower(-gamepad1.left_stick_y * leftTurn);
                BleftDrive.setPower(-gamepad1.left_stick_y * leftTurn);
                rightDrive.setPower(-gamepad1.left_stick_y * rightTurn);
                BrightDrive.setPower(-gamepad1.left_stick_y * rightTurn);
            }

            telemetry.addData("Can Move?", doMove);
//            telemetry.addData("Current Angle?", currentAngle);
//            telemetry.addData("Turn Adjust", adjustTurn);
//            telemetry.addData("Left Y", gamepad1.left_stick_y);
//            telemetry.addData("Left Encoder", leftDrive.getCurrentPosition());
//            telemetry.addData("Right Encoder", rightDrive.getCurrentPosition());
//            telemetry.addData("Time", currentTime);
            telemetry.addData("Sin X", currentTime);
            telemetry.addData("Sin Y", sinY);
            telemetry.addData("Derivative", derivOfFunct);
            telemetry.addData("Angle Of Line", slopeToAngle);
            telemetry.update();
        }


    }
}


