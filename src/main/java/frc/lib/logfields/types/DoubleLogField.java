package frc.lib.logfields.types;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import frc.lib.logfields.LogField;

public class DoubleLogField extends LogField<Double> {
    public DoubleLogField(String key, Supplier<Double> valueSupplier, double defaultValue) {
        super(key, valueSupplier, defaultValue);
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        table.put(key, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = table.getDouble(key, value);
    }
}
