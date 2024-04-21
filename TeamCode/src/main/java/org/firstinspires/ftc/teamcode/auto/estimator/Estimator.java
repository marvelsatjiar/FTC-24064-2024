package org.firstinspires.ftc.teamcode.auto.estimator;

import com.acmerobotics.roadrunner.Pose2d;

public class Estimator {
    // If extended: returns localizerPose if estimate is not available and a new pose if it is.
    // This class is generic and does not estimate.
    public Pose2d estimate(Pose2d localizerPose) {
        return localizerPose;
    }
}
