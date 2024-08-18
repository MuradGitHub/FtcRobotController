package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class TankTeleOp extends LinearOpMode {
    double slowModeCoefficent = 1;
    double arm2Power = 0;
    int target = 0;
    int error1;
    int error2;
    @Override
    public void runOpMode(){

        DcMotor drive1 = hardwareMap.dcMotor.get("drive_left");
        DcMotor drive2 = hardwareMap.dcMotor.get("drive_right");
        drive2.setDirection(DcMotorSimple.Direction.REVERSE);
        drive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor arm1 = hardwareMap.dcMotor.get("arm_left");
        DcMotor arm2 = hardwareMap.dcMotor.get("arm_right");
        arm2.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo claw = hardwareMap.servo.get("claw");
        claw.getController().pwmEnable();
        Servo wrist = hardwareMap.servo.get("wrist");
        wrist.getController().pwmEnable();

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        drive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wrist.setPosition(1);

        waitForStart();


        while (opModeIsActive()){
            if (gamepad1.left_bumper){
                slowModeCoefficent = 2;
            }
            else{
                slowModeCoefficent = 1;
            }
            error1 = target - arm1.getCurrentPosition();
            error2 = target - arm2.getCurrentPosition();
            arm1.setPower(error1 * 0.05);
            arm2.setPower(error2 * 0.05);

            if (isStopRequested()) return;

            if(gamepad1.right_stick_x < 0){
                drive1.setPower(0.5 / slowModeCoefficent);
                drive2.setPower(-0.5 / slowModeCoefficent);
            }
            else if (gamepad1.right_stick_x > 0){
                drive1.setPower(-0.5 / slowModeCoefficent);
                drive2.setPower(0.5 / slowModeCoefficent);
            }
            else if(-gamepad1.left_stick_y > 0){
                drive1.setPower(1 / slowModeCoefficent);
                drive2.setPower(1 / slowModeCoefficent);
            }
            else if(-gamepad1.left_stick_y < 0){
                drive1.setPower(-1 / slowModeCoefficent);
                drive2.setPower(-1 / slowModeCoefficent);
            }
            else{
                drive1.setPower(0);
                drive2.setPower(0);
            }

            if (gamepad1.right_trigger > 0.5){
                claw.setPosition(gamepad1.right_trigger);
                telemetry.addLine("claw pressed");
            }
            else{
                claw.setPosition(0.4);
            }

            if(gamepad1.x){
                wrist.setPosition(1);
                target = 0;
            }
            if(gamepad1.a){
                wrist.setPosition(0.40);
                target = 15;
                telemetry.addLine("a pressed");
            }

            if(gamepad1.y){
                wrist.setPosition(1);
                target = 500;
                telemetry.addLine("y pressed");
            }


            telemetry.addData("error1",error1);
            telemetry.addData("error2",error2);
            telemetry.addData("target",target);
            telemetry.addData("arm1 pos", arm1.getCurrentPosition());
            telemetry.addData("arm1 pos", arm2.getCurrentPosition());
            telemetry.addData("wrist pos", wrist.getPosition());
            telemetry.addData("wrist pos - Min", wrist.MIN_POSITION);
            telemetry.addData("wrist pos - Max", wrist.MAX_POSITION);
            telemetry.addData("claw pos", claw.getPosition());
            telemetry.addData("drive power", -gamepad1.left_stick_y);
            telemetry.update();
        }

    }

}
