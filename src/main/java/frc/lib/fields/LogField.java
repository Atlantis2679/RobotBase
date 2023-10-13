package frc.lib.fields;

import java.util.function.Supplier;

import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class LogField<T> implements Supplier<T>, LoggableInputs {
    protected final Supplier<T> valueSupplier;
    protected final String key;
    protected T value;

    public LogField(String key, Supplier<T> valueSupplier, T defaultValue) {
        this.key = key;
        this.valueSupplier = valueSupplier;
        this.value = defaultValue;
    }

    @Override
    public T get() {
        return value;
    }
}
