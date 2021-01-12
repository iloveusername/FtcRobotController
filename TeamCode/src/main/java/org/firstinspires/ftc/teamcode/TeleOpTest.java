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

@TeleOp(name="TeleOp Mark I", group="Basic")
public class TeleOpTest extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor BleftDrive = null;
    private DcMotor BrightDrive = null;

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
    boolean canDo = false;

    //Sets up a checkpoint system.
    double checkX = 0;
    double checkY = 0;

    //Sets Up The State Machine
    String roboState = "drive";


    @Override
    public void runOpMode() {

        //This sets up the gryoscope for use.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

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

        wheelDirection("up");

        while (opModeIsActive()) {

            resetDrive();

            //Gyro Stuff
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle = Math.round(-angles.firstAngle);

            //This just saved me sometime by having to type less.
            double stickX = gamepad1.left_stick_x;
            double stickY = -gamepad1.left_stick_y;

            //Pressing A will both break and save our position.
            if(gamepad1.a){
                if(doneTurn){
                    resetCount();
                    doneTurn = false;
                }

                //Since we only move in straight lines, we can take the total encoder count during that period, then multiple it by the sin or cosine of whatever angle we were facing to find X and Y values relative to origin.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = Math.round(-angles.firstAngle);
                currentY += (Math.cos(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                currentX += (Math.sin(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                currentY = (double) Math.round(currentY * 100) / 100;
                currentX = (double) Math.round(currentX * 100) / 100;

                //This just breaks.
                stickX = 0;
                stickY = 0;
                resetCount();

            }

            //Pressing B will reset our angle to what it started at.
            if(gamepad1.b){
                rotateToAngle(0);

            }

            //Rotate 90 Degrees left of right depending on input.
            if(gamepad1.x && gamepad1.dpad_left){
                rotateToAngle(currentAngle - 90);

            }
            if(gamepad1.x && gamepad1.dpad_right){
                rotateToAngle(currentAngle + 90);

            }

            //If we press the back button, we will return to origin.
            if(gamepad1.back && canDo){
                resetCount();
                goToOrigin(0.75);
            }

            //Drops a checkpoint.
            if(gamepad1.left_bumper && canDo){
                checkX = currentX;
                checkY = currentY;
            }

            //Resets your origin to where you currently are.
            if(gamepad1.left_stick_button && gamepad1.right_stick_button && canDo){
                currentX = 0;
                currentY = 0;
            }

            //Holding X cuts speed in half for percision and whatnot.
            if(gamepad1.x){
                stickX *= 2;
                stickY *= 2;
            }

            //Pressing the right bumper will bring us to our dropped checkpoint.
            if(gamepad1.right_bumper && canDo){
                resetCount();
                goToCoordinates(checkX, checkY,0.75);
            }

            //This is just a predetermined autonomous path we can take if we press Start. We can adjust to whatever we want.
            if(gamepad1.start && canDo){
                resetCount();
                goToCoordinates(0.5, 0.5,0.75);
                goToCoordinates(-0.5, 0.5,0.75);
                goToCoordinates(0, 0,0.75);
                rotateToAngle(0);
            }

            //Figures if we should be in a turning state, stand still, or driving state depending on our joystick's position.
            if(Math.abs(stickX) > 0.25){
                roboState = "turn";
            }
            else if(stickX == 0 && stickY == 0){
                if(doneTurn){
                    resetCount();
                }
                doneTurn = false;
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                currentAngle = Math.round(-angles.firstAngle);
                currentY += (Math.cos(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                currentX += (Math.sin(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                currentY = (double) Math.round(currentY * 100) / 100;
                currentX = (double) Math.round(currentX * 100) / 100;

                resetCount();
                canDo = true;
            }
            else{
                roboState = "drive";
                canDo = false;
            }


            //Depending on the state of the robot, do different things. This one is straightforward enough.
            switch (roboState){
                case "drive":
                    if(doneTurn){
                        resetCount();
                        doneTurn = false;
                    }
                    leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    trackEncoders = true;
                    wheelDirection("up");
                    leftDrive.setPower(stickY);
                    rightDrive.setPower(stickY);
                    BleftDrive.setPower(stickY);
                    BrightDrive.setPower(stickY);
                    break;
                case "turn":
                    if(trackEncoders){
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        currentAngle = Math.round(-angles.firstAngle);
                        currentY += (Math.cos(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                        currentX += (Math.sin(currentAngle * Math.PI/180) * (leftDrive.getCurrentPosition())) / meterToEncoder;
                        currentY = (double) Math.round(currentY * 100) / 100;
                        currentX = (double) Math.round(currentX * 100) / 100;
                        trackEncoders = false;
                    }
                    leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    wheelDirection("turnRight");
                    leftDrive.setPower(stickX);
                    rightDrive.setPower(stickX);
                    BleftDrive.setPower(stickX);
                    BrightDrive.setPower(stickX);
                    doneTurn = true;
                    break;
                case "dpad":
                    break;
            }


            //All of your telemetry desires can be fulfilled here.
            //telemetry.addData("Stick X", stickX);
            //telemetry.addData("Stick Y", stickY);
            telemetry.addData("Can Do?", canDo);
            telemetry.addData("Current Rot", currentAngle);
            telemetry.addData("Count Encoders?", trackEncoders);
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Checkpoint X", checkX);
            telemetry.addData("Checkpoint Y", checkY);
//            telemetry.addData("Angle Sine", Math.sin(currentAngle * Math.PI/180));
//            telemetry.addData("Angle Cosine", Math.cos(currentAngle * Math.PI/180));
//            telemetry.addData("State", roboState);
            telemetry.update();

        }
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

            //Turns the robot using encoders for accuracy. Adjust speed if you want.
            int encoderTurn = (int) Math.round(((AngleOfTri - currentAngle)*rotToEncoder));
            encoderTurn = Math.abs(encoderTurn);
            encoderDrive(encoderTurn, 0.5);

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

        //Turns the robot using encoders for accuracy. Adjust speed if you want.
        int encoderTurn = (int) Math.round(((AngleOfTri - currentAngle)*rotToEncoder));
        encoderTurn = Math.abs(encoderTurn);
        encoderDrive(encoderTurn, 0.75);

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
            //Turns the robot using encoders for accuracy. Adjust speed if you want.
            int encoderTurn = (int) Math.round(((AngleOfTri - currentAngle)*rotToEncoder));
            encoderTurn = Math.abs(encoderTurn);
            encoderDrive(encoderTurn, 0.75);

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

    public void rotateToAngle(double angle){
        boolean isDone = false;

        while(!isDone) {
            //Determine if we need to spin left or right.
            if (angle > currentAngle) {
                wheelDirection("turnRight");
            }
            if (angle < currentAngle) {
                wheelDirection("turnLeft");
            }

            //Turns the robot using encoders for accuracy. Adjust speed if you want.
            int encoderTurn = (int) Math.round(((angle - currentAngle) * rotToEncoder));
            encoderTurn = Math.abs(encoderTurn);
            encoderDrive(encoderTurn, 0.75);

            isDone = true;
        }
    }

    public void wheelDirection(String dir){
        switch (dir){
            case "up":
                leftDrive.setDirection(DcMotor.Direction.FORWARD);
                rightDrive.setDirection(DcMotor.Direction.REVERSE);
                BleftDrive.setDirection(DcMotor.Direction.FORWARD);
                BrightDrive.setDirection(DcMotor.Direction.REVERSE);
                break;
            case "down":
                leftDrive.setDirection(DcMotor.Direction.REVERSE);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);
                BleftDrive.setDirection(DcMotor.Direction.REVERSE);
                BrightDrive.setDirection(DcMotor.Direction.FORWARD);
                break;
            case "left":
                leftDrive.setDirection(DcMotor.Direction.REVERSE);
                rightDrive.setDirection(DcMotor.Direction.FORWARD);
                BleftDrive.setDirection(DcMotor.Direction.FORWARD);
                BrightDrive.setDirection(DcMotor.Direction.REVERSE);
                break;
            case "right":
                leftDrive.setDirection(DcMotor.Direction.FORWARD);
                rightDrive.setDirection(DcMotor.Direction.REVERSE);
                BleftDrive.setDirection(DcMotor.Direction.REVERSE);
                BrightDrive.setDirection(DcMotor.Direction.FORWARD);
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




