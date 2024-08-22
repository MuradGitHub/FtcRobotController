package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TwoWheelMotion {
    Telemetry telemetry;
    HardwareMap hardwareMap;
    DcMotor drive1;
    DcMotor drive2;
    DcMotor arm1;
    DcMotor arm2;
    Servo claw;
    Servo wrist;
    IMU imu;

    public TwoWheelMotion(HardwareMap hardwareMap,Telemetry telemetry){

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        drive1 = hardwareMap.dcMotor.get("drive_left");
        drive2 = hardwareMap.dcMotor.get("drive_right");
        arm1 = hardwareMap.dcMotor.get("arm_left");
        arm2 = hardwareMap.dcMotor.get("arm_right");
        claw = hardwareMap.servo.get("claw");
        wrist = hardwareMap.servo.get("wrist");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        drive2.setDirection(DcMotorSimple.Direction.REVERSE);
        drive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrist.setPosition(1);

    }

    private double kP = 0.1;
    public final double GEAR_RATIO = 20;
    public final double WHEEL_DIAMETER_MM = 90;
    public final double TICKS_PER_REVOLUTION = 28;
    public final double TICKS_PER_MM = (GEAR_RATIO * TICKS_PER_REVOLUTION) / WHEEL_DIAMETER_MM * Math.PI;

    public void moveForward(int mm){

        double driveError1 = mm - (drive1.getCurrentPosition() * TICKS_PER_MM);
        double driveError2 = mm - (drive1.getCurrentPosition() * TICKS_PER_MM);
        while(driveError1 > 5 || driveError2 > 5){
            driveError1 = mm - (drive1.getCurrentPosition() * TICKS_PER_MM);
            driveError2 = mm - (drive1.getCurrentPosition() * TICKS_PER_MM);

            drive1.setPower(kP * driveError1);
            drive2.setPower(kP * driveError2);

            telemetry.addData("drive1 pos", drive1.getCurrentPosition());
            telemetry.addData("drive2 pos", drive2.getCurrentPosition());
            telemetry.addData("drive1 error", driveError1);
            telemetry.addData("drive2 error", driveError2);
            telemetry.update();
        }
        drive1.setPower(0);
        drive2.setPower(0);

    }
    public void test(){
        while (true){

            drive1.setPower(0.5);
            drive2.setPower(0.5);
            telemetry.addData("ticks 1",drive1.getCurrentPosition());
            telemetry.addData("ticks 2",drive2.getCurrentPosition());


            telemetry.update();
        }

    }

}

