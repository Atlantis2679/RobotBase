package frc.lib.tuneables;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.sendableproperties.BooleanSendableProperty;
import frc.lib.tuneables.sendableproperties.NumberTunableProperty;
import frc.lib.tuneables.sendableproperties.StringTunableProperty;

public class TuneableSendableBuilder extends VisualizerSendableBuilder {
    private final SendableBuilder baseBuilder;
    private final LogFieldsTable fieldsTable;

    public TuneableSendableBuilder(SendableBuilder baseBuilder, LogFieldsTable fieldsTable) {
        super(baseBuilder);
        this.baseBuilder = baseBuilder;
        this.fieldsTable = fieldsTable;
    }

    @Override
    public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer setter) {
        new BooleanSendableProperty(key, getter, setter, fieldsTable, baseBuilder);
    }

    @Override
    public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter) {
        new NumberTunableProperty(key, getter, setter, fieldsTable, baseBuilder);
    }

    @Override
    public void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter) {
        new StringTunableProperty(key, getter, setter, fieldsTable, baseBuilder);
    }
}
