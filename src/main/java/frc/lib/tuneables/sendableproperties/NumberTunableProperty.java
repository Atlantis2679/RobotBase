package frc.lib.tuneables.sendableproperties;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;

public class NumberTunableProperty extends TuneableProperty {
    private final Supplier<double[]> field;
    private double[] valueFromNetwork = {};
    private final DoubleConsumer setter;

    public NumberTunableProperty(
            String key,
            DoubleSupplier getter,
            DoubleConsumer setter,
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
