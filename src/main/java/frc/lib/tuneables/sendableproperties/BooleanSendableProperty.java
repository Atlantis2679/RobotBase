package frc.lib.tuneables.sendableproperties;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;

public class BooleanSendableProperty implements SendableProperty {
    private final Supplier<boolean[]> field;
    private boolean[] valueFromNetwork = {};
    private final Consumer<Boolean> setter;

    public BooleanSendableProperty(
            String key,
            Supplier<Boolean> getter,
            Consumer<Boolean> setter,
            LogFieldsTable fieldsTable,
            SendableBuilder sendableBuilder) {
        this.setter = setter;

        field = fieldsTable.addBooleanArray(key, () -> {
            boolean[] newValue = valueFromNetwork;
            valueFromNetwork = new boolean[] {};
            return newValue;
        });

        sendableBuilder.addBooleanProperty(
                key,
                () -> {
                    boolean outputValue = getter.get();
                    fieldsTable.recordOutput(key, outputValue);
                    return outputValue;
                },
                value -> valueFromNetwork = new boolean[] { value });
    }

    @Override
    public void updateSetter() {
        if (field.get().length != 0) {
            setter.accept(field.get()[0]);
        }
    }
}
