package org.firstinspires.ftc.teamcode.oldtests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp(name="Hand Test", group="Basic")
public class HandTeleOp extends LinearOpMode {
    private DcMotor base = null;

    //This is a ratio for ratio things. About 2000 Encoder Ticks to a 90 Degree Turn. Default is ~22, Adjust to deal with encoder loss if needed. 1620 ticks for one meter, I think. I don't have a meter stick, so who really knows.
    static final double rotToEncoder = 2065 / 90;
    static final double meterToEncoder = 1620;

    @Override
    public void runOpMode() {

        waitForStart();

        base = hardwareMap.get(DcMotor.class, "base");

        base.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        base.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        base.setDirection(DcMotor.Direction.FORWARD);

        while (opModeIsActive()) {

            //This just saved me sometime by having to type less.
            double stickX = gamepad1.left_stick_x;
            double stickY = -gamepad1.left_stick_y;

            base.setPower(stickY);

            telemetry.addData("Base Pos", base.getCurrentPosition());
            telemetry.update();

        }
    }
}




