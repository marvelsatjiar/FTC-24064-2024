package org.firstinspires.ftc.teamcode.auto.estimator;

import com.acmerobotics.roadrunner.Pose2d;

public interface Estimator {
    // Returns localizerPose if estimate is not available and a new pose if it is.
    default Pose2d estimate(Pose2d localizerPose) {
        return localizerPose;
    }
}
