package org.firstinspires.ftc.teamcode.sensor;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.AveragingBuffer;

public final class HeadingIMU {
    private final IMU imu;

    private double heading, angularVel;
    private final AveragingBuffer headingBuffer, angularVelBuffer;

    public HeadingIMU(HardwareMap hardwareMap, String name, RevHubOrientationOnRobot imuOrientation) {
        imu = hardwareMap.get(IMU.class, name);
        imu.resetDeviceConfigurationForOpMode();
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(imuOrientation));

        headingBuffer = new AveragingBuffer(10);
        angularVelBuffer = new AveragingBuffer(10);
    }

    /**
     * Both values are put into a buffer to automatically be averaged with the last 10 values. This allows us to bypass IMU static and have a greater level of accuracy.
     */
    public void update() {
        heading = headingBuffer.put(imu.getRobotYawPitchRollAngles().getYaw(RADIANS));
        angularVel = angularVelBuffer.put(imu.getRobotAngularVelocity(RADIANS).zRotationRate);
    }

    public double getHeading() {
        return heading;
    }

    public double getAngularVel() {
        return angularVel;
    }
}
