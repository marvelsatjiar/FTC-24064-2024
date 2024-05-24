package org.firstinspires.ftc.teamcode.robot.test;

import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.mTelemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensor.HeadingIMU;
import org.firstinspires.ftc.teamcode.util.BulkReader;
import org.firstinspires.ftc.teamcode.util.LoopUtil;


@TeleOp(name = "Test Expansion Hub IMU", group = "Mechanism Test")
public final class TestExpansionHubIMU extends LinearOpMode {

    @Override
    public void runOpMode() {
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        HeadingIMU imu = new HeadingIMU(hardwareMap, "imu", new RevHubOrientationOnRobot(MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection));
        BulkReader bulkReader = new BulkReader(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            bulkReader.bulkRead();

            imu.update();

            mTelemetry.addData("Loop time (hertz)", LoopUtil.getLoopTimeInHertz());
            mTelemetry.addData("Heading", Math.toDegrees(imu.getHeading()));
            mTelemetry.update();
        }
    }
}
