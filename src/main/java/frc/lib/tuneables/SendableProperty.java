package frc.lib.tuneables;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class SendableProperty<T> {
    private final Supplier<T> field;
    private T lastFieldValue;
    private T valueFromNetwork;
    private final GetSet<T> valueHandler;

    public interface BuilderPropertyAdder<T> {
        void addProperty(String key, Supplier<T> getter, Consumer<T> setter);
    }

    public interface FieldCreator<T> {
        Supplier<T> createField(String key, Supplier<T> valueSupplier, T defaultValue);
    }

    public SendableProperty(
            String key,
            FieldCreator<T> fieldCreator,
            BuilderPropertyAdder<T> builderPropertyAdder,
            GetSet<T> valueHandler) {
        T defaultValue = valueHandler.get();
        valueFromNetwork = defaultValue;
        lastFieldValue = defaultValue;
        this.valueHandler = valueHandler;
        field = fieldCreator.createField(key, () -> valueFromNetwork, defaultValue);
        builderPropertyAdder.addProperty(key, valueHandler::get, value -> valueFromNetwork = value);
    }

    public void updateValueHandler() {
        T currFieldValue = field.get();
        if (lastFieldValue != currFieldValue) {
            valueHandler.set(currFieldValue);
            lastFieldValue = currFieldValue;
        }
    }
}
