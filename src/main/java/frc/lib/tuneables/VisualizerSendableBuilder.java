package frc.lib.tuneables;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.LongConsumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.Topic;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.function.FloatConsumer;
import edu.wpi.first.util.function.FloatSupplier;
import edu.wpi.first.util.sendable.SendableBuilder;

public class VisualizerSendableBuilder implements NTSendableBuilder {
    private final SendableBuilder baseBuilder;

    public VisualizerSendableBuilder(SendableBuilder baseBuilder) {
        this.baseBuilder = baseBuilder;
    }

    @Override
    public void close() throws Exception {
        baseBuilder.close();
    }

    @Override
    public void setSmartDashboardType(String type) {
        baseBuilder.setSmartDashboardType(type);
    }

    @Override
    public void setActuator(boolean value) {
        baseBuilder.setActuator(value);
    }

    @Override
    public void setSafeState(Runnable func) {
        baseBuilder.setSafeState(func);
    }

    @Override
    public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer setter) {
        baseBuilder.addBooleanProperty(key, getter, null);
    }

    @Override
    public void addIntegerProperty(String key, LongSupplier getter, LongConsumer setter) {
        baseBuilder.addIntegerProperty(key, getter, null);
    }

    @Override
    public void addFloatProperty(String key, FloatSupplier getter, FloatConsumer setter) {
        baseBuilder.addFloatProperty(key, getter, null);
    }

    @Override
    public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter) {
        baseBuilder.addDoubleProperty(key, getter, null);
    }

    @Override
    public void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter) {
        baseBuilder.addStringProperty(key, getter, null);
    }

    @Override
    public void addBooleanArrayProperty(String key, Supplier<boolean[]> getter, Consumer<boolean[]> setter) {
        baseBuilder.addBooleanProperty(key, null, null);
    }

    @Override
    public void addIntegerArrayProperty(String key, Supplier<long[]> getter, Consumer<long[]> setter) {
        baseBuilder.addIntegerArrayProperty(key, getter, null);
    }

    @Override
    public void addFloatArrayProperty(String key, Supplier<float[]> getter, Consumer<float[]> setter) {
        baseBuilder.addFloatArrayProperty(key, getter, null);
    }

    @Override
    public void addDoubleArrayProperty(String key, Supplier<double[]> getter, Consumer<double[]> setter) {
        baseBuilder.addDoubleArrayProperty(key, getter, null);
    }

    @Override
    public void addStringArrayProperty(String key, Supplier<String[]> getter, Consumer<String[]> setter) {
        baseBuilder.addStringArrayProperty(key, getter, null);
    }

    @Override
    public void addRawProperty(String key, String typeString, Supplier<byte[]> getter, Consumer<byte[]> setter) {
        baseBuilder.addRawProperty(key, typeString, getter, null);
    }

    @Override
    public BackendKind getBackendKind() {
        return baseBuilder.getBackendKind();
    }

    @Override
    public boolean isPublished() {
        return baseBuilder.isPublished();
    }

    @Override
    public void update() {
        baseBuilder.update();
    }

    @Override
    public void clearProperties() {
        baseBuilder.clearProperties();
    }

    @Override
    public void addCloseable(AutoCloseable closeable) {
        baseBuilder.addCloseable(closeable);
    }

    @Override
    public void setUpdateTable(Runnable func) {
        ((NTSendableBuilder) baseBuilder).setUpdateTable(func);
    }

    @Override
    public Topic getTopic(String key) {
        return ((NTSendableBuilder) baseBuilder).getTopic(key);
    }

    @Override
    public NetworkTable getTable() {
        return ((NTSendableBuilder) baseBuilder).getTable();
    }
}
