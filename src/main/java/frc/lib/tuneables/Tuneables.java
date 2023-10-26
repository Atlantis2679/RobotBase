package frc.lib.tuneables;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.tuneables.sendableproperties.TuneableProperty;

public class Tuneables {
    private static Map<String, Pair<Tuneable, BiConsumer<String, Sendable>>> waitingToEnableTuneables = new HashMap<>();
    private static boolean isEnabled = false;

    public static void add(String key, Tuneable tuneable, BiConsumer<String, Sendable> sendablePublisher) {
        if (isEnabled)
            publishTuneable(key, tuneable, sendablePublisher);
        else
            waitingToEnableTuneables.put(key, new Pair<>(tuneable, sendablePublisher));
    }

    public static void add(String key, Tuneable tuneable) {
        add(key, tuneable, SmartDashboard::putData);
    }

    public static void add(String key, Sendable sendable, BiConsumer<String, Sendable> sendablePublisher) {
        add(key, (Tuneable) sendable::initSendable, sendablePublisher);
    }

    public static void add(String key, Sendable sendable) {
        add(key, sendable, SmartDashboard::putData);
    }

    public static void enable() {
        if(!isEnabled){
            isEnabled = true;
            waitingToEnableTuneables.forEach((key, tuneableAndPublisher) -> {
                publishTuneable(key, tuneableAndPublisher.getFirst(), tuneableAndPublisher.getSecond());
            });
        }
    }

    private static void publishTuneable(String name, Tuneable tuneable, BiConsumer<String, Sendable> sendablePublisher) {
        sendablePublisher.accept(name, (builder) -> {
            tuneable.initTuneable(new TuneableBuilder(builder, name, sendablePublisher));
        });
    }

    public static void update() {
        TuneableProperty.updateAll();
    }
}
