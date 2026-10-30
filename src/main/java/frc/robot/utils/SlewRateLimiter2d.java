package frc.robot.utils;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;

public class SlewRateLimiter2d {
    private final double rateLimit;
    private Translation2d prevValue;
    private double prevTime;

    public SlewRateLimiter2d(double rateLimit) {
        this.rateLimit = rateLimit;
        reset(new Translation2d());
    }

    public Translation2d calculate(Translation2d input) {
        double now = Timer.getTimestamp();
        double maxStep = rateLimit * (now - prevTime);
        prevTime = now;

        Translation2d delta = input.minus(prevValue);
        double norm = delta.getNorm();
        if (norm > maxStep) delta = delta.times(maxStep / norm);

        prevValue = prevValue.plus(delta);
        return prevValue;
    }

    public void reset(Translation2d value) {
        prevValue = value;
        prevTime = Timer.getTimestamp();
    }
}