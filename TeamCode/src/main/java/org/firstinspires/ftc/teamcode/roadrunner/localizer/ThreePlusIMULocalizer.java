package org.firstinspires.ftc.teamcode.roadrunner.localizer;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.filters.dualfilter.ComplementaryFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.ComplementaryGains;
import org.firstinspires.ftc.teamcode.roadrunner.message.ThreePlusIMUInputsMessage;
import org.firstinspires.ftc.teamcode.sensor.HeadingIMU;

public final class ThreePlusIMULocalizer implements Localizer {
    public static class Params {
        public double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 1.0; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
        public double odoHeadingTrust = 0.2; // Trust coefficient in heading as provided by odometry
        public double odoHeadingVelTrust = 0.2; // Trust coefficient in heading velocity as provided by odometry
    }

    public static Params PARAMS = new Params();

    private final ComplementaryFilter headingFilter = new ComplementaryFilter(new ComplementaryGains(PARAMS.odoHeadingTrust));
    private final ComplementaryFilter headingVelFilter = new ComplementaryFilter(new ComplementaryGains(PARAMS.odoHeadingVelTrust));

    public final Encoder par0, par1, perp;
    public final HeadingIMU imu;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private Rotation2d lastHeading;

    public final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public ThreePlusIMULocalizer(HardwareMap hardwareMap, HeadingIMU imu, double inPerTick) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: reverse encoder directions if needed
        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

        this.imu = imu;

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_PLUS_IMU_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        double rawHeading = imu.getHeading();
        double rawHeadingVel = imu.getAngularVel();

        FlightRecorder.write("THREE_PLUS_IMU_INPUTS", new ThreePlusIMUInputsMessage(par0PosVel, par1PosVel, perpPosVel, rawHeading, rawHeadingVel));

        Rotation2d heading = Rotation2d.exp(rawHeading);

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
            headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        }
        lastRawHeadingVel = rawHeadingVel;
        double headingVel = headingVelOffset + rawHeadingVel;

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingFilter.calculate((par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks), headingDelta),
                        headingVelFilter.calculate((par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks), headingVel),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        return twist;
    }
}
