package frc.lib.tuneables;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;

public class TuneablesTable {
    private static final List<TuneablesTable> createdTables = new ArrayList<>();

    private final List<SendableProperyCreator> sendablePropertyCreators = new ArrayList<>();
    private final List<SendableProperty<?>> sendablePropertis = new ArrayList<>();

    public TuneablesTable() {
        createdTables.add(this);
    }

    public static void updateAll() {
        for (TuneablesTable tuneablesTable : createdTables) {
            tuneablesTable.updateProperties();
        }
    }

    private interface SendableProperyCreator {
        SendableProperty<?> createProperty(LogFieldsTable fieldsTable, SendableBuilder sendableBuilder);
    }

    public void addBoolean(String key, Supplier<Boolean> getter, Consumer<Boolean> setter) {
        sendablePropertyCreators.add((fieldsTable, sendableBuilder) -> new SendableProperty<Boolean>(
                key,
                fieldsTable::addBoolean,
                (builderKey, builderGetter, builderSetter) -> sendableBuilder.addBooleanProperty(
                        builderKey,
                        builderGetter::get,
                        builderSetter::accept),
                new SuppliedGetSet<>(getter, setter)));
    }

    public void publish(String key, BiConsumer<String, Sendable> sendablePublisher) {
        System.out.println("nowwww");
        sendablePublisher.accept(key, new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                System.out.println("here we gooo!");
                LogFieldsTable fieldsTable = new LogFieldsTable("Tuneables/" + key);
                for (SendableProperyCreator sendableProperyCreator : sendablePropertyCreators) {
                    sendablePropertis.add(sendableProperyCreator.createProperty(fieldsTable, builder));
                }
            }
        });
    }

    public void updateProperties() {
        for (SendableProperty<?> sendableProperty : sendablePropertis) {
            sendableProperty.updateValueHandler();
        }
    }

    private class SuppliedGetSet<T> implements GetSet<T> {
        private final Supplier<T> getter;
        private final Consumer<T> setter;

        public SuppliedGetSet(Supplier<T> getter, Consumer<T> setter) {
            this.getter = getter;
            this.setter = setter;
        }

        @Override
        public T get() {
            return getter.get();
        }

        @Override
        public void set(T value) {
            setter.accept(value);
        }
    }
}
