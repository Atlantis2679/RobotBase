package frc.lib.tuneables.sendableproperties;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;

public class NumberSendableProperty implements SendableProperty {
    private final Supplier<double[]> field;
    private double[] valueFromNetwork = {};
    private final Consumer<Double> setter;

    public NumberSendableProperty(
            String key,
            DoubleSupplier getter,
            Consumer<Double> setter,
            LogFieldsTable fieldsTable,
            SendableBuilder sendableBuilder) {
        this.setter = setter;

        field = fieldsTable.addDoubleArray(key, () -> {
            double[] newValue = valueFromNetwork;
            valueFromNetwork = new double[] {};
            return newValue;
        });

        sendableBuilder.addDoubleProperty(
                key,
                () -> {
                    double outputValue = getter.getAsDouble();
                    fieldsTable.recordOutput(key, outputValue);
                    return outputValue;
                },
                value -> valueFromNetwork = new double[] { value });
    }

    @Override
    public void updateSetter() {
        if (field.get().length != 0) {
            setter.accept(field.get()[0]);
        }
    }
}
