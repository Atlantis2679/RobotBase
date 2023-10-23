package frc.lib.tuneables.wrappers;

import java.util.function.BiConsumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.tuneables.Tuneable;

public class PIDControllerTuneable extends PIDController implements Tuneable {
    public PIDControllerTuneable(double kp, double ki, double kd) {
        super(kp, ki, kd);
    }

    public PIDControllerTuneable(double kp, double ki, double kd, double period) {
        super(kp, ki, kd, period);
    }

    @Override
    public void publish(String key, BiConsumer<String, Sendable> sendablePublisher) {
        Tuneable.fromSendable(super::initSendable).publish(key);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        DriverStation.reportWarning(
                "Tried to publish tuneable PID controller to dashboard without using publish().",
                new Error().getStackTrace());
    }
}
