package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class NLEFT extends LinearOpMode{
    Ninjabot robot;

    @Override
    public void runOpMode() {
        robot = new Ninjabot(hardwareMap, this);

        int FORWARD = 1;
        int BACKWARD = 3;
        int ROTATE_LEFT = 5;
        int ROTATE_RIGHT = 6;
        int TANK_LEFT= 7;
        int TANK_RIGHT= 8;

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
// stall motors
        robot.leftDrive.setTargetPosition(0);
        robot.rightDrive.setTargetPosition(0);
// zero out the motors counters

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setPower(0.3);
        robot.rightDrive.setPower(0.3);


        //move before inizilise here

        waitForStart();
//set power for all wheels indefinitely
        //Put moves here

        //testing
        robot.claw.setPosition(0.5);
        while (robot.inRange(robot.claw.getPosition(),0.5,0.1) && opModeIsActive()){
            sleep(200);
        }
        telemetry.addData("done","and done");
        robot.claw.setPosition(1);
        while (robot.inRange(robot.claw.getPosition(),1,0.1) && opModeIsActive()){
            sleep(200);
        }
    }
}
