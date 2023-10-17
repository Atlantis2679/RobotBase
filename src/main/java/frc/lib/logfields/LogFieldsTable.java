package frc.lib.logfields;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class LogFieldsTable implements LoggableInputs {
    private final static ArrayList<LogFieldsTable> createdTables = new ArrayList<>();

    private final String name;
    private final String prefix;
    private final Logger logger = Logger.getInstance();
    private final List<LoggableInputs> fields = new ArrayList<>();
    private Runnable periodicBeforeFields = null;

    public LogFieldsTable(String name) {
        this.name = name;
        prefix = name + "/";
        createdTables.add(this);
    }

    public static void updateAll() {
        for (LogFieldsTable fieldsTable : createdTables) {
            if (fieldsTable.periodicBeforeFields != null && !Logger.getInstance().hasReplaySource()) {
                fieldsTable.periodicBeforeFields.run();
            }
            Logger.getInstance().processInputs(fieldsTable.name, fieldsTable);
        }
    }

    public LogFieldsTable getSubTable(String name) {
        return new LogFieldsTable(prefix + name);
    }

    @Override
    public void toLog(LogTable table) {
        for (LoggableInputs field : fields) {
            field.toLog(table);
        }
    }

    @Override
    public void fromLog(LogTable table) {
        for (LoggableInputs field : fields) {
            field.fromLog(table);
        }
    }

    public void setPeriodicBeforeFields(Runnable periodicRunnable) {
        if(fields.isEmpty()){
            periodicRunnable.run(); // so in the init cycle this will still run before the fields.
        }
        this.periodicBeforeFields = periodicRunnable;
    }

    private <T> LogField<T> registerField(LogField<T> field) {
        fields.add(field);
        Logger.getInstance().processInputs(name, field); // so in the init cycle the value will still update and logged/replayed.
        return field;
    }

    public Supplier<byte[]> addRaw(
            String name,
            Supplier<byte[]> valueSupplier,
            byte[] defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getRaw(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<byte[]> addRaw(String name, Supplier<byte[]> valueSupplier) {
        return addRaw(name, valueSupplier, new byte[] {});
    }

    public Supplier<Boolean> addBoolean(
            String name,
            Supplier<Boolean> valueSupplier,
            boolean defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getBoolean(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<Boolean> addBoolean(String name, Supplier<Boolean> valueSupplier) {
        return addBoolean(name, valueSupplier, false);
    }

    public Supplier<Long> addInteger(
            String name,
            Supplier<Long> valueSupplier,
            long defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getInteger(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<Long> addInteger(String name, Supplier<Long> valueSupplier) {
        return addInteger(name, valueSupplier, 0);
    }

    public Supplier<Float> addFloat(
            String name,
            Supplier<Float> valueSupplier,
            float defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getFloat(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<Float> addFloat(String name, Supplier<Float> valueSupplier) {
        return addFloat(name, valueSupplier, 0);
    }

    public Supplier<Double> addDouble(
            String name,
            Supplier<Double> valueSupplier,
            double defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getDouble(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<Double> addDouble(String name, Supplier<Double> valueSupplier) {
        return addDouble(name, valueSupplier, 0);
    }

    public Supplier<String> addString(
            String name,
            Supplier<String> valueSupplier,
            String defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getString(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<String> addString(String name, Supplier<String> valueSupplier) {
        return addString(name, valueSupplier, "");
    }

    public Supplier<boolean[]> addBooleanArray(
            String name,
            Supplier<boolean[]> valueSupplier,
            boolean[] defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getBooleanArray(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<boolean[]> addBooleanArray(String name, Supplier<boolean[]> valueSupplier) {
        return addBooleanArray(name, valueSupplier, new boolean[] {});
    }

    public Supplier<long[]> addIntegerArray(
            String name,
            Supplier<long[]> valueSupplier,
            long[] defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getIntegerArray(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<long[]> addIntegerArray(String name, Supplier<long[]> valueSupplier) {
        return addIntegerArray(name, valueSupplier, new long[] {});
    }

    public Supplier<float[]> addFloatArray(
            String name,
            Supplier<float[]> valueSupplier,
            float[] defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getFloatArray(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<float[]> addFloatArray(String name, Supplier<float[]> valueSupplier) {
        return addFloatArray(name, valueSupplier, new float[] {});
    }

    public Supplier<double[]> addDoubleArray(
            String name,
            Supplier<double[]> valueSupplier,
            double[] defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getDoubleArray(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<double[]> addDoubleArray(String name, Supplier<double[]> valueSupplier) {
        return addDoubleArray(name, valueSupplier, new double[] {});
    }

    public Supplier<String[]> addStringArray(
            String name,
            Supplier<String[]> valueSupplier,
            String[] defaultValue) {
        return registerField(new LogField<>(
                valueSupplier,
                (table) -> table.getStringArray(name, defaultValue),
                (table, value) -> table.put(name, value)));
    }

    public Supplier<String[]> addStringArray(String name, Supplier<String[]> valueSupplier) {
        return addStringArray(name, valueSupplier, new String[] {});
    }

    public void recordOutput(String name, byte[] value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, boolean value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, long value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, float value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, double value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, String value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, boolean[] value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, long[] value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, float[] value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, double[] value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, String[] value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, Pose2d... value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, Pose3d... value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, Trajectory value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, SwerveModuleState... value) {
        logger.recordOutput(prefix + name, value);
    }

    public void recordOutput(String name, Mechanism2d value) {
        logger.recordOutput(prefix + name, value);
    }
}