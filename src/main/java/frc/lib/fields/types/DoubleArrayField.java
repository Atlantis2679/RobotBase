package frc.lib.fields.types;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import frc.lib.fields.FieldBase;

public class DoubleArrayField extends FieldBase<double[]> {
    public DoubleArrayField(String key, Supplier<double[]> valueSupplier, double[] defaultValue) {
        super(key, valueSupplier, defaultValue);
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        table.put(key, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = table.getDoubleArray(key, value);
    }
}
