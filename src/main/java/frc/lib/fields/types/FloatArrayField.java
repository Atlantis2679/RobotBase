package frc.lib.fields.types;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import frc.lib.fields.FieldBase;

public class FloatArrayField extends FieldBase<float[]> {
    public FloatArrayField(String key, Supplier<float[]> valueSupplier, float[] defaultValue) {
        super(key, valueSupplier, defaultValue);
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        table.put(key, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = table.getFloatArray(key, value);
    }
}
