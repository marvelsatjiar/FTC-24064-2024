package org.firstinspires.ftc.teamcode.control.filter.singlefilter;

public final class NoFilter implements Filter {

    @Override
    public double calculate(double newValue) {
        return newValue;
    }

    @Override
    public void reset() {

    }
}
