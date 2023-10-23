package frc.lib.tuneables.sendableproperties;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;

public class BooleanSendableProperty extends TuneableProperty {
    private final Supplier<boolean[]> field;
    private boolean[] valueFromNetwork = {};
    private final BooleanConsumer setter;

    public BooleanSendableProperty(
            String key,
            BooleanSupplier getter,
            BooleanConsumer setter,
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
                    boolean outputValue = getter.getAsBoolean();
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
