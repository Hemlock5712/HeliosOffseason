package frc.robot.util;

public class MathUtilities {
    public static double interpolate(double lowerKey, double upperKey, double lowerValue, double upperValue, double currentValue)  {
        return lowerValue + ((currentValue - lowerKey) / (upperKey - lowerKey)) * (upperValue - lowerValue);
    }
}
