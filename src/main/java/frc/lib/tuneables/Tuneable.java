package frc.lib.tuneables;

import java.util.function.BiConsumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface Tuneable {
    void publish(String key, BiConsumer<String, Sendable> sendablePublisher);

    default void publish(String key) {
        publish(key, SmartDashboard::putData);
    }
}
