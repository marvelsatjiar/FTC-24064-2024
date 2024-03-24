package org.firstinspires.ftc.teamcode.robot.centerstage;

import static org.firstinspires.ftc.teamcode.robot.centerstage.Robot.mTelemetry;
import static org.firstinspires.ftc.teamcode.util.SimpleServoPivot.getGoBildaServo;
import static java.lang.Math.max;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Rollers {
    private final MotorEx intake;
    private final SimpleServo deployableRoller;


    public static double ANGLE_DEPLOYABLE = 90;
    public static double ANGLE_MIN_DEPLOYABLE = 2;
    public double setPoint = ANGLE_DEPLOYABLE;

    public Rollers(HardwareMap hardwareMap) {
        deployableRoller = getGoBildaServo(hardwareMap, "roller1");
        intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
    }

    public void intake(double power) {
        intake.set(power);
    }

    public double intakePower() {
        return intake.get();
    }

    public void setDeployableWithTrigger(double trigger) {
        setDeployable(trigger < 0 ? max(ANGLE_MIN_DEPLOYABLE, ANGLE_DEPLOYABLE + ANGLE_DEPLOYABLE * trigger) : ANGLE_DEPLOYABLE);
    }

    public void resetDeployable() {
        setDeployable(ANGLE_DEPLOYABLE);
    }

    public void setDeployable(double angle) {
        setPoint = angle;
    }

    public void run() {
        deployableRoller.turnToAngle(setPoint);
    }

    public void printTelemetry() {
        mTelemetry.addData("Roller angle (degrees)", setPoint);
    }
}
