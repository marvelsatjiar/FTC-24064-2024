package org.firstinspires.ftc.teamcode.robot.mechanismtest;

import static org.firstinspires.ftc.teamcode.robot.centerstage.Robot.mTelemetry;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.drivetrain.AprilTagDrive;
import org.firstinspires.ftc.teamcode.sensor.vision.AprilTagSensor;

@TeleOp(group = "Single mechanism test")
public final class TestAprilTagDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        mTelemetry = new MultipleTelemetry(telemetry);

        AprilTagDrive drive = new AprilTagDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        drive.createAprilTagSensor();

        waitForStart();

        while(opModeIsActive()) {
            drive.updatePoseEstimate();

            mTelemetry.addData("x (inches)", drive.pose.position.x);
            mTelemetry.addData("y (inches)", drive.pose.position.y);
            mTelemetry.addData("Heading (degrees)", Math.toDegrees(drive.pose.heading.toDouble()));
            mTelemetry.update();

            sleep(20);
        }
    }
}
