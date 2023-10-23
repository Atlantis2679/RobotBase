package frc.lib.tuneables;

import java.util.function.BiConsumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logfields.LogFieldsTable;

public interface Tuneable {
    static String LOG_PREFIX = "Tuneables/";

    void publish(String key, BiConsumer<String, Sendable> sendablePublisher);

    default void publish(String key) {
        publish(key, SmartDashboard::putData);
    }

    static Tuneable fromSendable(Sendable sendable) {
        return (key, sendablePublisher) -> {
            sendablePublisher.accept(key, (builder) -> {
                LogFieldsTable fieldsTable = new LogFieldsTable(LOG_PREFIX + key);
                sendable.initSendable(new TuneableSendableBuilder(builder, fieldsTable));
            });
        };
    }
}
