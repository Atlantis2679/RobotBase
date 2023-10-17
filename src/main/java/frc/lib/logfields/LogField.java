package frc.lib.logfields;

import java.util.function.BiConsumer;
import java.util.function.Function;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LogField<T> implements Supplier<T>, LoggableInputs {
    private final Supplier<T> valueSupplier;
    private final Function<LogTable, T> getFromLog;
    private final BiConsumer<LogTable, T> putInLog;
    private T value;

    public LogField(
            Supplier<T> valueSupplier,
            Function<LogTable, T> getFromLog,
            BiConsumer<LogTable, T> putInLog) {
        this.valueSupplier = valueSupplier;
        this.getFromLog = getFromLog;
        this.putInLog = putInLog;
    }

    @Override
    public T get() {
        return value;
    }

    @Override
    public void toLog(LogTable table) {
        value = valueSupplier.get();
        putInLog.accept(table, value);
    }

    @Override
    public void fromLog(LogTable table) {
        value = getFromLog.apply(table);
    }
}
