package frc.robot.util;

public class MathUtil {
    // gives the positive distance between two values over a cyclical range (like
    // distance between two angles)
    public static double getCyclicalDistance(double a, double b, double range) {
        a = mod(a, range);
        b = mod(b, range);

        double ret = Math.abs(a - b);
        ret = Math.min(ret, a + (range - b));
        ret = Math.min(ret, b + (range - a));

        return ret;
    }

    public static double mod(double a, double b) { //Returns a positive mod of a values in [-b, b]
        return ((a % b) + b) % b;
    }
    
    public static double clipToZero(double val, double min) { //Returns zero if val is in [-min, min], otherwise returns val
        return val > min || val < -min ? val : 0;
    }
}