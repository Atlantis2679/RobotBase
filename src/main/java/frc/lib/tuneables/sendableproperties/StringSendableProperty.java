package frc.lib.tuneables.sendableproperties;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;

public class StringSendableProperty implements SendableProperty {
    private final Supplier<String[]> field;
    private String[] valueFromNetwork = {};
    private final Consumer<String> setter;

    public StringSendableProperty(
            String key,
            Supplier<String> getter,
            Consumer<String> setter,
            LogFieldsTable fieldsTable,
            SendableBuilder sendableBuilder) {
        this.setter = setter;

        field = fieldsTable.addStringArray(key, () -> {
            String[] newValue = valueFromNetwork;
            valueFromNetwork = new String[] {};
            return newValue;
        });

        sendableBuilder.addStringProperty(
                key,
                () -> {
                    String outputValue = getter.get();
                    fieldsTable.recordOutput(key, outputValue);
                    return outputValue;
                },
                value -> valueFromNetwork = new String[] { value });
    }

    @Override
    public void updateSetter() {
        if (field.get().length != 0) {
            setter.accept(field.get()[0]);
        }
    }
}
