package frc.lib.logfields.types;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import frc.lib.logfields.LogField;

public class StringArrayLogField extends LogField<String[]> {
    public StringArrayLogField(String key, Supplier<String[]> valueSupplier, String[] defaultValue) {
        super(key, valueSupplier, defaultValue);
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        table.put(key, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = table.getStringArray(key, value);
    }
}
