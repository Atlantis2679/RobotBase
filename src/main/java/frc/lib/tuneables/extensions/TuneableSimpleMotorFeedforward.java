package frc.lib.tuneables.extensions;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;

public class TuneableSimpleMotorFeedforward implements Tuneable {
    private SimpleMotorFeedforward baseFeedforward;

    public TuneableSimpleMotorFeedforward(double ks, double kv, double ka) {
        baseFeedforward = new SimpleMotorFeedforward(ks, kv, ka);
    }

    public double calculate(
            double velocity,
            double acceleration) {
        return baseFeedforward.calculate(velocity, acceleration);
    }

    public double calculate(double velocity) {
        return baseFeedforward.calculate(velocity);
    }

    public double maxAchievableVelocity(double maxVoltage, double acceleration) {
        return baseFeedforward.maxAchievableAcceleration(maxVoltage, acceleration);
    }

    public double minAchievableVelocity(double maxVoltage, double acceleration) {
        return baseFeedforward.minAchievableAcceleration(maxVoltage, acceleration);
    }

    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return baseFeedforward.maxAchievableAcceleration(maxVoltage, velocity);
    }

    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return baseFeedforward.minAchievableAcceleration(maxVoltage, velocity);
    }

    public void setKS(double newKS) {
        baseFeedforward = new SimpleMotorFeedforward(newKS, baseFeedforward.kv, baseFeedforward.ka);
    }

    public void setKV(double newKV) {
        baseFeedforward = new SimpleMotorFeedforward(baseFeedforward.ks, newKV, baseFeedforward.ka);
    }

    public void setKA(double newKA) {
        baseFeedforward = new SimpleMotorFeedforward(baseFeedforward.ks, baseFeedforward.kv, newKA);
    }

    public SimpleMotorFeedforward getArmFeedforward() {
        return baseFeedforward;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.setSendableType(SendableType.LIST);
        builder.addDoubleProperty("kS", () -> baseFeedforward.ks, this::setKS);
        builder.addDoubleProperty("kV", () -> baseFeedforward.kv, this::setKV);
        builder.addDoubleProperty("kA", () -> baseFeedforward.ka, this::setKA);
    }

}
