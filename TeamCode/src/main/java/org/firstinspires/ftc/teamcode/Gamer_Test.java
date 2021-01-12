package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Gamer Stuff Test", group="Basic")
public class Gamer_Test extends LinearOpMode{
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

    boolean doMove = true;

    double adjustTurn = 0;

    double currentAngle = 0;

    double encoderStep = -1;






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


            curveWithEncoders(2,Math.PI,1);
            sleep(10000);
        }


    }



    public void curveWithEncoders(double a, double b, double c){
        boolean isDone = false;

        double x= -1;
        double sinY;
        double derivOfFunct;
        double slopeToAngle;
        double turningDistance;
        double leftRatio = 0;
        double rightRatio = 0;

        while(!isDone && opModeIsActive()){

            sinY = (a*Math.asin(x))/b;
            derivOfFunct = a/(b*Math.sqrt(1-(x*x)));
            slopeToAngle = 90 - (Math.atan(derivOfFunct)*180/Math.PI);

            //Gyro Stuff
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = Math.round(-angles.firstAngle);
            turningDistance = slopeToAngle - currentAngle;

            if(turningDistance < 20){
                leftRatio = 1;
                rightRatio = (turningDistance/-20) + 1.1;
                if(rightRatio > 1 || turningDistance < 0){
                    rightRatio = 1;
                }

            }
            if(turningDistance > 20){
                leftRatio = (turningDistance - 20)/20;
                rightRatio = 0.1;
            }


            encoderDriveCurve(1000, 0.5, leftRatio, rightRatio);


            x += 0.1;

            telemetry.addData("Left", leftRatio);
            telemetry.addData("Right", rightRatio);
            telemetry.addData("Sin X", x);
            telemetry.addData("Sin Y", sinY);
            telemetry.addData("Derivative", derivOfFunct);
            telemetry.addData("Angle Of Line", slopeToAngle);
            telemetry.addData("Current Angle", currentAngle);
            telemetry.update();

            if(x > 0){
                sleep(20000);
            }

        }


    }

    public void encoderDriveCurve(int desiredEncoder, double desiredSpeed, double left, double right){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setTargetPosition((int) (desiredEncoder * left));
        rightDrive.setTargetPosition((int) (desiredEncoder * right));
        BleftDrive.setTargetPosition((int) (desiredEncoder * left));
        BrightDrive.setTargetPosition((int) (desiredEncoder * right));

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
}


