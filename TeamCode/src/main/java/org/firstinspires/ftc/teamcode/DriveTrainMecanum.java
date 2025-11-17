package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Mat;

@TeleOp(name = "Mecanum")
public class DriveTrainMecanum extends LinearOpMode {

    public DcMotorEx frontRight, backRight, frontLeft, backLeft;
    private IMU imu;
    public void runOpMode(){
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontLeft =  hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        frontRight.setDirection(Direction.REVERSE);
        backRight.setDirection(Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
           )));

        waitForStart();
        imu.resetYaw();

        while (opModeIsActive()){

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = gamepad1.right_stick_x;

            double HeadingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double heading = Math.toRadians(-HeadingDeg);

            double rotatedX = x * Math.cos(heading) - y * Math.sin(heading);
            double rotatedY = y * Math.sin(heading) + y * Math.cos(heading);

            double leftFront = rotatedY + rotatedX + r;
            double rightFront = rotatedY - rotatedX - r;
            double leftBack = rotatedY - rotatedX + r;
            double rightBack = rotatedY + rotatedX - r;

            double max = Math.max(1.0, Math.max(
                    Math.abs(leftFront),
                    Math.max(Math.abs(rightFront),
                            Math.max(Math.abs(leftBack), Math.abs(rightBack)))
            ));

            leftFront /= max; rightFront /= max; leftBack /=max; rightBack /= max;

            frontRight.setPower(rightFront);
            frontLeft.setPower(leftFront);
            backLeft.setPower(leftBack);
            backRight.setPower(rightBack);

        }

    }


}
