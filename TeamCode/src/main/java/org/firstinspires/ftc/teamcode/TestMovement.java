package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TestMovement extends LinearOpMode {
    @Override
    public void runOpMode(){
        TwoWheelMotion twoWheelMotion = new TwoWheelMotion(hardwareMap, telemetry);
        waitForStart();
        twoWheelMotion.moveForward(300);
        while (opModeIsActive()) {sleep(20);}
    }
}
