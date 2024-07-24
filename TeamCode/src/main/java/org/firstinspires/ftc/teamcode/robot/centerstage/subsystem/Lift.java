package org.firstinspires.ftc.teamcode.robot.centerstage.subsystem;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;
import static org.firstinspires.ftc.teamcode.robot.centerstage.opmode.MainTeleOp.mTelemetry;
import static org.firstinspires.ftc.teamcode.robot.centerstage.subsystem.Robot.maxVoltage;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.control.filter.singlefilter.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.FeedforwardGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.control.motion.State;

@Config
public final class Lift {

    /**
     * A PIDGains object being set to certain values (tweak these numbers!!)
     */
    public static PIDGains pidGains = new PIDGains(
            0.005,
            0.002,
            0.0001,
            Double.POSITIVE_INFINITY
    );

    public static FeedforwardGains feedforwardGains = new FeedforwardGains(
            0.01,
            0.01
    );

    private double lastKp = pidGains.kP;

    // Sets the filter for PID outputs and constrains overshoots with controlling (also tweak!!)
    public static LowPassGains filterGains = new LowPassGains(0, 2);

    /**
     * Sets the constants for the positions, conversions, etc
     * Remember to set these constants correctly! (in ticks)
     */
    public static double
            AUTON_ROW_HEIGHT = 600,
            MAX_MOTOR_TICKS = 2350,
            MIN_MOTOR_TICKS = -5,
            kG = 0.011065,
            PERCENT_OVERSHOOT = 0,
            JOYSTICK_MULTIPLIER = 40;

    private final MotorEx[] motors;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private final VoltageSensor batteryVoltageSensor;

    private State currentState = new State();

    private double targetTicks = 0;
    private int setPoint = -1;

    /**
     * Constructor of Lift class; Sets variables with hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Lift(HardwareMap hardwareMap) {
        MotorEx leader = new MotorEx(hardwareMap, "leader", RPM_435);
        MotorEx follower = new MotorEx(hardwareMap, "follower", RPM_435);

        leader.encoder.reset();
        follower.encoder.reset();

        leader.setInverted(false);
        follower.setInverted(false);

        motors = new MotorEx[]{leader, follower};
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void setWithStick(double stick) {
        targetTicks = min(MAX_MOTOR_TICKS, max(MIN_MOTOR_TICKS, targetTicks + stick * JOYSTICK_MULTIPLIER));
        setPoint = targetTicks == MIN_MOTOR_TICKS ? -1 : 0;
        controller.setTarget(new State(targetTicks));
    }

    public int getSetPoint() {
        return setPoint;
    }

    public void setToAutonHeight(double offset) {
        setPoint = 0;
        controller.setTarget(new State(AUTON_ROW_HEIGHT + offset));
    }

    public void retract() {
        setPoint = -1;
        controller.setTarget(new State(MIN_MOTOR_TICKS));
    }

    /**
     * Sets the three variables; The currentState sets to the position of the motor; Other two set the constants of the gains
     * Calls another run() method that calculates the motor output proportionally and doesn't compensate for power
     */
    public void run() {
        currentState = new State(0.5 * (-motors[0].encoder.getPosition() + motors[1].encoder.getPosition()));
        if (lastKp != pidGains.kP) {
//            pidGains.computeKd(feedforwardGains, PERCENT_OVERSHOOT);
            lastKp = pidGains.kP;
        }
        controller.setGains(pidGains);
        derivFilter.setGains(filterGains);

        run(controller.calculate(currentState), false);
    }

    public void run(double motorPower) {
        run(motorPower, true);
    }

    /**
     * Checks if the voltage is to be saved, if true, use scalar to modify motor power
     * @param motorPower; The power that is set to be used to set the motor's power
     * @param voltageCompensate; Boolean that is used if battery is low, and if it needs to compensate (save)
     */
    public void run(double motorPower, boolean voltageCompensate) {
        double scalar = maxVoltage / batteryVoltageSensor.getVoltage();

        if (voltageCompensate) motorPower *= scalar;

        for (MotorEx motor : motors) motor.set(motorPower + kG * scalar);
    }

    public void printTelemetry() {
        mTelemetry.addData("Target position (ticks)", targetTicks);
    }

    public void printNumericalTelemetry() {
        mTelemetry.addData("Current position (ticks)", currentState.x);
        mTelemetry.addData("Error derivative (ticks/s)", controller.getFilteredErrorDerivative());
        mTelemetry.addData("Error (ticks)", controller.getErrorIntegral());
        mTelemetry.addData("kD (computed)", pidGains.kD);
    }
}
