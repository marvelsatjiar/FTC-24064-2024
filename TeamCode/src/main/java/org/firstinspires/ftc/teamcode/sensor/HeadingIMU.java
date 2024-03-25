package org.firstinspires.ftc.teamcode.sensor;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.control.filters.singlefilter.MovingAverageFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.MovingAverageGains;

import javax.annotation.concurrent.GuardedBy;

public final class HeadingIMU {
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    private final IMU imu;
    private Thread imuThread;

    private double heading, angularVel;
    private final MovingAverageFilter headingFilter, angularVelFilter;

    public HeadingIMU(HardwareMap hardwareMap, String name, RevHubOrientationOnRobot imuOrientation) {
        synchronized (imuLock) {
            imu = hardwareMap.get(IMU.class, name);
            imu.resetDeviceConfigurationForOpMode();
            imu.resetYaw();
            imu.initialize(new IMU.Parameters(imuOrientation));
        }

        MovingAverageGains gains = new MovingAverageGains(10);
        headingFilter = new MovingAverageFilter(gains);
        angularVelFilter = new MovingAverageFilter(gains);
    }

    public void startIMUThread(LinearOpMode opMode) {
        imuThread = new Thread(() -> {
            while (!opMode.isStopRequested()) {
                synchronized (imuLock) {
                    update();
                }
            }
        });
        imuThread.start();
    }

    /**
     * Both values are put into a buffer to automatically be averaged with the last 10 values. This allows us to bypass IMU static and have a greater level of accuracy.
     */
    private void update() {
        heading = headingFilter.calculate(imu.getRobotYawPitchRollAngles().getYaw(RADIANS));
        angularVel = angularVelFilter.calculate(imu.getRobotAngularVelocity(RADIANS).zRotationRate);
    }

    public double getHeading() {
        return heading;
    }

    public double getAngularVel() {
        return angularVel;
    }
}
