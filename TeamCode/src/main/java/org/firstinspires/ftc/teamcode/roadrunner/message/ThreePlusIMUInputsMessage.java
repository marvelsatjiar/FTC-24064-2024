package org.firstinspires.ftc.teamcode.roadrunner.message;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class ThreePlusIMUInputsMessage {
    public long timestamp;
    public PositionVelocityPair par0;
    public PositionVelocityPair par1;
    public PositionVelocityPair perp;
    public double heading;
    public double headingVel;

    public ThreePlusIMUInputsMessage(PositionVelocityPair par0, PositionVelocityPair par1, PositionVelocityPair perp, double heading, double headingVel) {
        this.timestamp = System.nanoTime();
        this.par0 = par0;
        this.par1 = par1;
        this.perp = perp;
        {
            this.heading = heading;
            this.headingVel = headingVel;
        }
    }
}
