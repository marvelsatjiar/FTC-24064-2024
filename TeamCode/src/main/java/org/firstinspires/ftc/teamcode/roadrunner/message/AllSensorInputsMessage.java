package org.firstinspires.ftc.teamcode.roadrunner.message;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class AllSensorInputsMessage {
    public long timestamp;
    public PositionVelocityPair par0;
    public PositionVelocityPair par1;
    public PositionVelocityPair perp;
    public double heading;
    public double headingVel;
    public double estimateX;
    public double estimateY;
    public double estimateHeading;

    public AllSensorInputsMessage(PositionVelocityPair par0, PositionVelocityPair par1, PositionVelocityPair perp, double heading, double headingVel, Pose2d tagEstimate) {
        this.timestamp = System.nanoTime();
        this.par0 = par0;
        this.par1 = par1;
        this.perp = perp;
        {
            this.heading = heading;
            this.headingVel = headingVel;
        }
        if (tagEstimate != null) {
            estimateX = tagEstimate.position.x;
            estimateY = tagEstimate.position.y;
            estimateHeading = tagEstimate.heading.toDouble();
        }
    }
}
