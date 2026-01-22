package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.sun.tools.doclint.Entity.not;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MotorcontrolerGamepad")
public class Gamepad extends LinearOpMode {

    public DcMotorEx motor0, motor1, motor2, motor3, IntakeMotor, ShooterMotor1, ShooterMotor2;
    private Limelight3A limelight;

    private IMU imu;

    @Override
    public void runOpMode() {
        // --- CONFIGURACIÓN DE HARDWARE ---
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);

        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));



        motor0 = hardwareMap.get(DcMotorEx.class, "FLM");
        motor1 = hardwareMap.get(DcMotorEx.class, "FRM");
        motor2 = hardwareMap.get(DcMotorEx.class, "BLM");
        motor3 = hardwareMap.get(DcMotorEx.class, "BRM");

        motor0.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IM");
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShooterMotor1 = hardwareMap.get(DcMotorEx.class, "SM1");
        ShooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "SM2");
        ShooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        ShooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- ESPERAR A QUE EMPIECE ---
        waitForStart();

        // Iniciar Limelight después del Start
        limelight.start();

        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()){
                Pose3D botPose = llResult.getBotpose_MT2();
                double distance = getDistanceFromTarget(llResult.getTa());
                telemetry.addData("Distance", distance);
                telemetry.addData("Target X", llResult.getTx());
                telemetry.addData("Botpose", botPose.toString());
            }

            // --- LÓGICA DE MOVIMIENTO (MECANUM) ---
            double forward = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;
            double distance = getDistanceFromTarget(llResult.getTargetingLatency());
            double shootDistance = 180;



            motor0.setPower(forward + turn + strafe);
            motor1.setPower(forward - turn - strafe);
            motor2.setPower(forward + turn - strafe);
            motor3.setPower(forward - turn + strafe);

            if (gamepad1.right_trigger > 0) {
                ShooterMotor1.setPower(1);
                ShooterMotor2.setPower(1);
            }
            else if (gamepad1.left_trigger > 0)
            {
                ShooterMotor1.setPower(0.85);
                ShooterMotor2.setPower(0.85);
            }
            else
            {
                ShooterMotor1.setPower(0);
                ShooterMotor2.setPower(0);
            }

            if (gamepad1.left_bumper) IntakeMotor.setPower(-0.9);
            else if (gamepad1.right_bumper) IntakeMotor.setPower(0.9);
            else IntakeMotor.setPower(0);

            if (gamepad1.square)
            {
                if(distance < shootDistance)
                {
                    motor0.setPower(-1);
                    motor1.setPower(-1);
                    motor2.setPower(-1);
                    motor3.setPower(-1);
                }
                else if(distance > shootDistance)
                {
                    motor0.setPower(1);
                    motor1.setPower(1);
                    motor2.setPower(1);
                    motor3.setPower(1);

                }
            }


            telemetry.update();
        }
    }

    public double getDistanceFromTarget(double ta) {
        double K = 179.0; // cm
        return K / Math.sqrt(ta);
    }
}