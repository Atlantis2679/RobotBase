package frc.lib.tuneables.extensions;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;

public class TuneableArmFeedforward implements Tuneable {
    private ArmFeedforward armFeedforward;

    public TuneableArmFeedforward(double ks, double kg, double kv) {
        armFeedforward = new ArmFeedforward(ks, kg, kv);
    }

    public TuneableArmFeedforward(double ks, double kg, double kv, double ka) {
        armFeedforward = new ArmFeedforward(ks, kg, kv, ka);
    }

    public double calculate(
            double positionRadians,
            double velocityRadPerSec,
            double accelRadPerSecSquared) {
        return armFeedforward.calculate(positionRadians, velocityRadPerSec);
    }

    public double calculate(double positionRadians, double velocity) {
        return armFeedforward.calculate(positionRadians, velocity);
    }

    public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
        return armFeedforward.maxAchievableAcceleration(maxVoltage, angle, acceleration);
    }

    public double minAchievableVelocity(double maxVoltage, double angle, double acceleration) {
        return armFeedforward.minAchievableAcceleration(maxVoltage, angle, acceleration);
    }

    public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
        return armFeedforward.maxAchievableAcceleration(maxVoltage, angle, velocity);
    }

    public double minAchievableAcceleration(double maxVoltage, double angle, double velocity) {
        return armFeedforward.minAchievableAcceleration(maxVoltage, angle, velocity);
    }

    public void setKS(double value) {
        armFeedforward = new ArmFeedforward(value, armFeedforward.kg, armFeedforward.kv, armFeedforward.ka);
    }

    public void setKG(double value) {
        armFeedforward = new ArmFeedforward(armFeedforward.ks, value, armFeedforward.kv, armFeedforward.ka);
    }

    public void setKV(double value) {
        armFeedforward = new ArmFeedforward(armFeedforward.ks, armFeedforward.kg, value, armFeedforward.ka);
    }

    public void setKA(double value) {
        armFeedforward = new ArmFeedforward(armFeedforward.ks, armFeedforward.kg, armFeedforward.kv, value);
    }

    public ArmFeedforward getArmFeedforward() {
        return armFeedforward;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.setSendableType(SendableType.LIST);
        builder.addDoubleProperty("kS", () -> armFeedforward.ks, this::setKS);
        builder.addDoubleProperty("kG", () -> armFeedforward.kg, this::setKG);
        builder.addDoubleProperty("kV", () -> armFeedforward.kv, this::setKV);
        builder.addDoubleProperty("kA", () -> armFeedforward.ka, this::setKA);
    }

}
