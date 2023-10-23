package frc.lib.tuneables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.refvalues.BooleanRefValue;
import frc.lib.refvalues.DoubleRefValue;
import frc.lib.refvalues.RefValue;
import frc.lib.tuneables.sendableproperties.BooleanSendableProperty;
import frc.lib.tuneables.sendableproperties.NumberTunableProperty;
import frc.lib.tuneables.sendableproperties.TuneableProperty;
import frc.lib.tuneables.sendableproperties.StringTunableProperty;

public class TuneablesTable implements Tuneable {
    private final SendableType sendableType;
    private final List<PropertyNode> propertyNodes = new ArrayList<>();
    private final Map<String, Tuneable> tuneablesNode = new HashMap<>();

    private final List<TuneableProperty> sendableProperties = new ArrayList<>();

    private interface PropertyNode {
        TuneableProperty createSendableProperty(LogFieldsTable fieldsTable, SendableBuilder sendableBuilder);
    }

    public TuneablesTable(SendableType sendableType) {
        this.sendableType = sendableType;
    }

    public void addVisualizer(String name, Sendable sendable) {
        addTuneable(name, (key, sendablePublisher) -> {
            sendablePublisher.accept(key, builder -> {
                sendable.initSendable(new VisualizerSendableBuilder(builder));
            });
        });
    }

    public void addTuneable(String name, Tuneable tuneable) {
        tuneablesNode.put(name, tuneable);
    }

    public void addBoolean(String key, BooleanSupplier getter, BooleanConsumer setter) {
        propertyNodes.add((fields, builder) -> new BooleanSendableProperty(key, getter, setter, fields, builder));
    }

    public BooleanRefValue addBoolean(String key, boolean defaultValue) {
        BooleanRefValue refValue = new BooleanRefValue(defaultValue);
        addBoolean(key, refValue::get, refValue::set);
        return refValue;
    }

    public void addNumber(String key, DoubleSupplier getter, DoubleConsumer setter) {
        propertyNodes.add((fields, builder) -> new NumberTunableProperty(key, getter, setter, fields, builder));
    }

    public DoubleRefValue addNumber(String key, Double defaultValue) {
        DoubleRefValue refValue = new DoubleRefValue(defaultValue);
        addNumber(key, refValue::get, refValue::set);
        return refValue;
    }

    public void addString(String key, Supplier<String> getter, Consumer<String> setter) {
        propertyNodes.add((fields, builder) -> new StringTunableProperty(key, getter, setter, fields, builder));
    }

    public RefValue<String> addString(String key, String defaultValue) {
        RefValue<String> refValue = new RefValue<String>(defaultValue);
        addString(key, refValue::get, refValue::set);
        return refValue;
    }

    @Override
    public void publish(String key, BiConsumer<String, Sendable> sendablePublisher) {
        sendablePublisher.accept(key, builder -> {
            if (sendableType != SendableType.NONE) {
                builder.setSmartDashboardType(sendableType.getStringType());
            }
            LogFieldsTable fieldsTable = new LogFieldsTable(LOG_PREFIX + key);
            for (PropertyNode propertyNode : propertyNodes) {
                sendableProperties.add(propertyNode.createSendableProperty(fieldsTable, builder));
            }
        });

        tuneablesNode.forEach((name, tuneable) -> {
            tuneable.publish(key + "/" + name, sendablePublisher);
        });
    }
}
