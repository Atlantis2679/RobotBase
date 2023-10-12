package frc.lib.fields.types;

import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;

import frc.lib.fields.FieldBase;

public class StringField extends FieldBase<String> {    
    public StringField(String key, Supplier<String> valueSupplier, String defaultValue) {
        super(key, valueSupplier, defaultValue);
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        table.put(key, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = table.getString(key, value);
    }
}
