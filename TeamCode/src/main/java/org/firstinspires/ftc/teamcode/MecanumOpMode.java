package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumOpMode")
public class MecanumOpMode extends LinearOpMode {

    DcMotorEx FL, FR, BL, BR, SM, IM;

    @Override
    public void runOpMode() {

        FL = hardwareMap.get(DcMotorEx.class, "FLM");
        FR = hardwareMap.get(DcMotorEx.class, "FRM");
        BL = hardwareMap.get(DcMotorEx.class, "BLM");
        BR = hardwareMap.get(DcMotorEx.class, "BRM");
        SM = hardwareMap.get(DcMotorEx.class,"SM");
        IM = hardwareMap.get(DcMotorEx.class,"IM");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        SM.setDirection(DcMotorSimple.Direction.REVERSE);
        IM.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        while (opModeIsActive()) {

            double forward = gamepad1.right_trigger - gamepad1.left_trigger;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            double flPower = forward + turn + strafe;
            double frPower = forward - turn - strafe;
            double blPower = forward + turn - strafe;
            double brPower = forward - turn + strafe;

            double max = Math.max(
                    Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower))
            );

            if (max > 1.0) {
                flPower /= max;
                frPower /= max;
                blPower /= max;
                brPower /= max;
            }

            if (gamepad1.right_bumper){
                IM.setPower(0.9);
            } else if (gamepad1.left_bumper){
                IM.setPower(-0.9);
            } else {
                IM.setPower(0);
            }

            double power = gamepad1.left_stick_y / max;
            SM.setPower(power);





            FL.setPower(flPower);
            FR.setPower(frPower);
            BL.setPower(blPower);
            BR.setPower(brPower);
        }
    }
}
