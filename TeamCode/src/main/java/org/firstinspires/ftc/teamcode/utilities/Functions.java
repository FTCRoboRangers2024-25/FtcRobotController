package org.firstinspires.ftc.teamcode.utilities;

public class Functions {
    public static double LinearInterpolation(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double Clamp(double min, double max, double value) {
        if (value < min) return min;
        else if (value > max) return max;
        else return value;
    }

    public static double Clamp01(double value) {
        return Clamp(0, 1, value);
    }
}
