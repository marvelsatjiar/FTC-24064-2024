package org.firstinspires.ftc.teamcode.roadrunner.localizer;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.roadrunner.message.ThreePlusAprilTagInputsMessage;
import org.firstinspires.ftc.teamcode.sensor.vision.AprilTagSensor;

public final class ThreePlusAprilTagLocalizer implements Localizer {
    public static class Params {
        public double par0YTicks = 0.0; // y position of the first parallel encoder (in tick units)
        public double par1YTicks = 1.0; // y position of the second parallel encoder (in tick units)
        public double perpXTicks = 0.0; // x position of the perpendicular encoder (in tick units)
        public double odoPositionTrust = 0.4; // Trust coefficient in position as provided by odometry
    }

    public static Params PARAMS = new Params();

    private final ComplementaryFilter positionFilter = new ComplementaryFilter(new ComplementaryGains(PARAMS.odoPositionTrust));

    public final Encoder par0, par1, perp;
    public AprilTagSensor aprilTag;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private Rotation2d lastHeading;
    private final Pose2d pose;

    public final double inPerTick;

    private final HardwareMap hardwareMap;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;

    public ThreePlusAprilTagLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d pose) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par0")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "par1")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "perp")));

        // TODO: reverse encoder directions if needed
        //   par0.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        this.hardwareMap = hardwareMap;
        this.pose = pose;

        FlightRecorder.write("THREE_PLUS_APRIL_TAG_PARAMS", PARAMS);
    }

    public void createAprilTagSensor() {
        aprilTag = new AprilTagSensor(hardwareMap);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        Pose2d tagEstimate = null;
        if (aprilTag != null) {
            tagEstimate = aprilTag.getPoseEstimate();
        }

        FlightRecorder.write("THREE_PLUS_APRIL_TAG_INPUTS", new ThreePlusAprilTagInputsMessage(par0PosVel, par1PosVel, perpPosVel, tagEstimate));

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            Twist2dDual<Time> init = new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );

            // Not sure if this is necessary. Possibly just adding 0 to everything because I've got no idea what .constant() does. See the end of the method.
            pose.plus(init.value());

            return init;
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        double xDelta = (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks);
        double yDelta = (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta);

        if (tagEstimate != null) {
            double estimateXDelta = tagEstimate.position.x - pose.position.x;
            double estimateYDelta = tagEstimate.position.y - pose.position.y;

            xDelta = positionFilter.calculate(xDelta, estimateXDelta);
            yDelta = positionFilter.calculate(yDelta, estimateYDelta);
        }

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                xDelta,
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                yDelta,
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;
        // The localizer keeps a copy of the position to find delta position in the case of sensing a tag.
        // This essentially acts as a "lastTagEstimate" even though we aren't bound to get the estimate every loop.
        pose.plus(twist.value());

        return twist;
    }
}
