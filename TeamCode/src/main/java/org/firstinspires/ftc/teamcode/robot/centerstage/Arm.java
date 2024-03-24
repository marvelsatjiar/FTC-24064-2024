package org.firstinspires.ftc.teamcode.robot.centerstage;

import static org.firstinspires.ftc.teamcode.robot.centerstage.Robot.mTelemetry;
import static org.firstinspires.ftc.teamcode.util.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.util.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

@Config
public final class Arm {
    final SimpleServoPivot armPivot, flap, wrist;

    boolean flapTimerCondition = false;

    public final ElapsedTime flapTimer = new ElapsedTime();

    // TODO Measure TIME_RETRACT_ARM
    public static double
            ANGLE_COLLECTING = 103,
            ANGLE_DEPOSITING = 190,
            ANGLE_OPEN_FLAP = 90,
            ANGLE_CLOSED_FLAP = 0,
            TIME_DEPOSIT_1_PIXEL = 0.18,
            ANGLE_WRIST_UNDEPLOYED = 60,
            ANGLE_WRIST_DEPLOYED = 45;

    public Arm(HardwareMap hardwareMap) {
        flap = new SimpleServoPivot(ANGLE_OPEN_FLAP, ANGLE_CLOSED_FLAP, getGoBildaServo(hardwareMap, "flap"));

        wrist = new SimpleServoPivot(
                ANGLE_WRIST_UNDEPLOYED,
                ANGLE_WRIST_DEPLOYED,
                getGoBildaServo(hardwareMap, "wrist1"),
                getReversedServo(getGoBildaServo(hardwareMap, "wrist2"))
        );

        armPivot = new SimpleServoPivot(
                ANGLE_COLLECTING,
                ANGLE_DEPOSITING,
                getGoBildaServo(hardwareMap, "arm1"),
                getReversedServo(getGoBildaServo(hardwareMap, "arm2"))
        );
    }

    public void toggleArm() {
        armPivot.toggle();
        wrist.toggle();
    }

    public void setArm(boolean isDepositing) {
        armPivot.setActivated(isDepositing);
    }

    public void toggleFlap() {
        flap.toggle();

        flapTimerCondition = !flap.isActivated();
        if (flapTimerCondition) {
            flapTimer.reset();
        }
    }

    public void setFlap(boolean isClosed) {
        flap.setActivated(isClosed);
    }

    public boolean isArmActivated() {
        return armPivot.isActivated();
    }

    public void run() {
        flap.updateAngles(ANGLE_OPEN_FLAP, ANGLE_CLOSED_FLAP);
        armPivot.updateAngles(ANGLE_COLLECTING, ANGLE_DEPOSITING);
        wrist.updateAngles(ANGLE_WRIST_UNDEPLOYED, ANGLE_WRIST_DEPLOYED);
        flap.run();
        armPivot.run();
        wrist.run();
    }

    public void printTelemetry() {
        mTelemetry.addData("flap is", (flap.isActivated() ? "closed" : "open"));
        mTelemetry.addData("Arm is", "running to " + (armPivot.isActivated() ? "deposit" : "collect"));
    }
}