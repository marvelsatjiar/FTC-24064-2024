package org.firstinspires.ftc.teamcode.auto.estimator;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.filter.dualfilter.ComplementaryFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ComplementaryGains;
import org.firstinspires.ftc.teamcode.robot.drivetrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.sensor.HeadingIMU;

public class IMUEstimator implements CreateIMUEstimator {
    public static class Params {
        // For example, 0.4 means localizer will be weighted 40% while April Tag estimate will be weighted 60%
        public double localizerTrust = 0.4;
    }

    public static Params PARAMS = new Params();

    private final HeadingIMU imu;
    private double lastImuHeading;

    private final ComplementaryFilter filter = new ComplementaryFilter(new ComplementaryGains(PARAMS.localizerTrust));

    public IMUEstimator(HardwareMap hardwareMap) {
        imu = new HeadingIMU(hardwareMap, "imu", new RevHubOrientationOnRobot(MecanumDrive.PARAMS.logoFacingDirection, MecanumDrive.PARAMS.usbFacingDirection));
    }

    @Override
    public Pose2d estimate(Pose2d localizerPose) {
        Pose2d pose = localizerPose;
        double imuHeading = imu.getHeading();

        if (imuHeading != lastImuHeading) {
            pose = new Pose2d(localizerPose.position, filter.calculate(localizerPose.heading.toDouble(), imuHeading));
        }

        lastImuHeading = imuHeading;
        return pose;
    }

    @Override
    public void startIMUThread(LinearOpMode opMode) {
        imu.startIMUThread(opMode);
    }
}
