package frc.lib.fields.types;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import frc.lib.fields.LogField;

public class BooleanArrayLogField extends LogField<boolean[]> {
    public BooleanArrayLogField(String key, Supplier<boolean[]> valueSupplier, boolean[] defaultValue) {
        super(key, valueSupplier, defaultValue);
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        table.put(key, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = table.getBooleanArray(key, value);
    }
}
