package org.firstinspires.ftc.teamcode.robot.mechanismtest;

import static org.firstinspires.ftc.teamcode.robot.centerstage.Robot.mTelemetry;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensor.vision.AprilTagSensor;

@TeleOp(group = "Single mechanism test")
public final class TestAprilTagLocalization extends LinearOpMode {

    @Override
    public void runOpMode() {
        mTelemetry = new MultipleTelemetry(telemetry);

        AprilTagSensor aprilTag = new AprilTagSensor(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            mTelemetry.addData("Pose estimate (inches)", aprilTag.getPoseEstimate());
            mTelemetry.update();

            sleep(20);
        }
    }
}
