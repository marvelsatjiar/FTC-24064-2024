package org.firstinspires.ftc.teamcode.auto.estimator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public interface IIMUEstimator extends Estimator {
    void startIMUThread(LinearOpMode opMode);
}
