package org.firstinspires.ftc.teamcode.control.filters.dualfilter;

public class NoDualFilter implements DualFilter {
    @Override
    public double calculate(double value1, double value2) {
        return Double.NaN;
    }

    @Override
    public void reset() {

    }
}
