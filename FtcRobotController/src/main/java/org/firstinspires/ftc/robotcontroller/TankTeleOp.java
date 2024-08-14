package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class TankTeleOp extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor drive1 = hardwareMap.dcMotor.get("drive1");
        DcMotor drive2 = hardwareMap.dcMotor.get("drive2");
        drive2.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor arm1 = hardwareMap.dcMotor.get("arm1");
        DcMotor arm2 = hardwareMap.dcMotor.get("arm2");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        drive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        while (opModeIsActive()){
            if (isStopRequested()) return;
        }

    }

}
