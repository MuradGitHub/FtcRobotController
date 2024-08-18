package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class TankWithClaw extends LinearOpMode {

    // Calculate the COUNTS_PER_mm for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV  = 28 ;   // Rev
    static final double     DRIVE_GEAR_REDUCTION  = 20.0 ; //
    static final double     WHEEL_DIAMETER_mm     = 90.0 ; // For figuring circumference
    static final double     COUNTS_PER_mm         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_mm * Math.PI);


    protected ElapsedTime runtime    = null;

    IMU                   imu        = null;
    DcMotor               driveLeft  = null;
    DcMotor               driveRight = null;
    DcMotor               armLeft    = null;
    DcMotor               armRight   = null;
    Servo                 wrist      = null;
    Servo                 claw       = null;

    public TankWithClaw() {
        super();
        initialize();
    }

    protected void initialize() {

        runtime   = new ElapsedTime();

        // To drive forward, most robots need the motor on one side to be reversed, because the
        // axles point in opposite directions. When run, this OpMode should start both motors
        // driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.
        // Gear Reduction or 90 Deg drives may require direction flips
        driveLeft  = hardwareMap.dcMotor.get("drive_left");
        driveLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveRight = hardwareMap.dcMotor.get("drive_right");
        driveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armLeft    = hardwareMap.dcMotor.get("arm_left");
        armLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        armRight   = hardwareMap.dcMotor.get("arm_right");

        claw       = hardwareMap.servo.get("claw");
        claw.getController().pwmEnable();

        wrist      = hardwareMap.servo.get("wrist");
        wrist.getController().pwmEnable();

        imu        = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Send telemetry message to indicate successful initialization, Encoder reset
        telemetry.addData("TankWithClaw:initialize", "complete");
        telemetry.addData("Left Drive ", driveLeft.getCurrentPosition());
        telemetry.addData("Right Drive", driveRight.getCurrentPosition());
        telemetry.update();
    }

}
