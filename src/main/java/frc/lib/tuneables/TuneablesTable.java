package frc.lib.tuneables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.RefValue;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.sendableproperties.BooleanSendableProperty;
import frc.lib.tuneables.sendableproperties.NumberSendableProperty;
import frc.lib.tuneables.sendableproperties.SendableProperty;
import frc.lib.tuneables.sendableproperties.StringSendableProperty;

public class TuneablesTable implements Tuneable {
    private static final List<TuneablesTable> createdTables = new ArrayList<>();

    private final SendableType sendableType;
    private final List<PropertyNode> propertyNodes = new ArrayList<>();
    private final Map<String, Tuneable> tuneablesNode = new HashMap<>();

    private final List<SendableProperty> sendableProperties = new ArrayList<>();

    private interface PropertyNode {
        SendableProperty createSendableProperty(LogFieldsTable fieldsTable, SendableBuilder sendableBuilder);
    }

    public TuneablesTable(SendableType sendableType) {
        this.sendableType = sendableType;
        createdTables.add(this);
    }

    public static void updateAll() {
        for (TuneablesTable tuneablesTable : createdTables) {
            tuneablesTable.updateProperties();
        }
    }

    public void addTuneable(String name, Tuneable tuneable) {
        tuneablesNode.put(name, tuneable);
    }

    public void addBoolean(String key, Supplier<Boolean> getter, Consumer<Boolean> setter) {
        propertyNodes.add((fields, builder) -> new BooleanSendableProperty(key, getter, setter, fields, builder));
    }

    public RefValue<Boolean> addBoolean(String key, boolean defaultValue) {
        RefValue<Boolean> refValue = new RefValue<Boolean>(defaultValue);
        addBoolean(key, refValue::get, refValue::set);
        return refValue;
    }

    public void addNumber(String key, Supplier<Double> getter, Consumer<Double> setter) {
        propertyNodes.add((fields, builder) -> new NumberSendableProperty(key, getter, setter, fields, builder));
    }

    public RefValue<Double> addNumber(String key, Double defaultValue) {
        RefValue<Double> refValue = new RefValue<Double>(defaultValue);
        addNumber(key, refValue::get, refValue::set);
        return refValue;
    }

    public void addString(String key, Supplier<String> getter, Consumer<String> setter) {
        propertyNodes.add((fields, builder) -> new StringSendableProperty(key, getter, setter, fields, builder));
    }

    public RefValue<String> addString(String key, String defaultValue) {
        RefValue<String> refValue = new RefValue<String>(defaultValue);
        addString(key, refValue::get, refValue::set);
        return refValue;
    }

    @Override
    public void publish(String key, BiConsumer<String, Sendable> sendablePublisher) {
        sendablePublisher.accept(key, new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                if(sendableType != SendableType.NONE){
                    builder.setSmartDashboardType(sendableType.getStringType());
                }
                LogFieldsTable fieldsTable = new LogFieldsTable("Tuneables/" + key);
                for (PropertyNode propertyNode : propertyNodes) {
                    sendableProperties.add(propertyNode.createSendableProperty(fieldsTable, builder));
                }
            }
        });

        tuneablesNode.forEach((name, tuneable) -> {
            tuneable.publish(key + "/" + name, sendablePublisher);
        });
    }

    public void updateProperties() {
        for (SendableProperty sendableProperty : sendableProperties) {
            sendableProperty.updateSetter();
        }
    }
}
