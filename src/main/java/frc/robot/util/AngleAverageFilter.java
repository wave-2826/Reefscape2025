package frc.robot.util;

import edu.wpi.first.math.filter.LinearFilter;

public class AngleAverageFilter {
    private final LinearFilter xFilter;
    private final LinearFilter yFilter;

    public AngleAverageFilter(int taps) {
        this.xFilter = LinearFilter.movingAverage(taps);
        this.yFilter = LinearFilter.movingAverage(taps);
    }

    public void reset() {
        xFilter.reset();
        yFilter.reset();
    }

    public double calculate(double input) {
        double x = Math.cos(input);
        double y = Math.sin(input);

        double xFiltered = xFilter.calculate(x);
        double yFiltered = yFilter.calculate(y);

        return Math.atan2(yFiltered, xFiltered);
    }
}
