package frc.lib.fields;

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

import frc.lib.fields.types.BooleanArrayField;
import frc.lib.fields.types.BooleanField;
import frc.lib.fields.types.DoubleArrayField;
import frc.lib.fields.types.DoubleField;
import frc.lib.fields.types.FloatArrayField;
import frc.lib.fields.types.FloatField;
import frc.lib.fields.types.IntegerArrayField;
import frc.lib.fields.types.IntegerField;
import frc.lib.fields.types.RawField;
import frc.lib.fields.types.StringArrayField;
import frc.lib.fields.types.StringField;
import frc.robot.Robot;

public class FieldsTable implements LoggableInputs {
    private final String prefix;
    private final Logger logger = Logger.getInstance();
    private final List<LoggableInputs> fields = new ArrayList<>();
    private Runnable periodicBeforeFields = null;

    public FieldsTable(String name) {
        Robot.registerPeriodic(() -> {
            if (periodicBeforeFields != null && !Logger.getInstance().hasReplaySource()) {
                periodicBeforeFields.run();
            }
            Logger.getInstance().processInputs(name, this);
        });
        prefix = name + "/";
    }

    public FieldsTable getSubTable(String name) {
        return new FieldsTable(prefix + name);
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
        this.periodicBeforeFields = periodicRunnable;
    }

    public Supplier<byte[]> addRaw(
            String name,
            Supplier<byte[]> valueSupplier,
            byte[] defaultValue) {
        RawField field = new RawField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<byte[]> addRaw(String name, Supplier<byte[]> valueSupplier) {
        return addRaw(name, valueSupplier, new byte[] {});
    }

    public Supplier<Boolean> addBoolean(
            String name,
            Supplier<Boolean> valueSupplier,
            boolean defaultValue) {
        BooleanField field = new BooleanField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<Boolean> addBoolean(String name, Supplier<Boolean> valueSupplier) {
        return addBoolean(name, valueSupplier, false);
    }

    public Supplier<Long> addInteger(
            String name,
            Supplier<Long> valueSupplier,
            long defaultValue) {
        IntegerField field = new IntegerField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<Long> addInteger(String name, Supplier<Long> valueSupplier) {
        return addInteger(name, valueSupplier, 0);
    }

    public Supplier<Float> addFloat(
            String name,
            Supplier<Float> valueSupplier,
            float defaultValue) {
        FloatField field = new FloatField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<Float> addFloat(String name, Supplier<Float> valueSupplier) {
        return addFloat(name, valueSupplier, 0);
    }

    public Supplier<Double> addDouble(
            String name,
            Supplier<Double> valueSupplier,
            double defaultValue) {
        DoubleField field = new DoubleField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<Double> addDouble(String name, Supplier<Double> valueSupplier) {
        return addDouble(name, valueSupplier, 0);
    }

    public Supplier<String> addString(
            String name,
            Supplier<String> valueSupplier,
            String defaultValue) {
        StringField field = new StringField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<String> addString(String name, Supplier<String> valueSupplier) {
        return addString(name, valueSupplier, "");
    }

    public Supplier<boolean[]> addBooleanArray(
            String name,
            Supplier<boolean[]> valueSupplier,
            boolean[] defaultValue) {
        BooleanArrayField field = new BooleanArrayField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<boolean[]> addBooleanArray(String name, Supplier<boolean[]> valueSupplier) {
        return addBooleanArray(name, valueSupplier, new boolean[] {});
    }

    public Supplier<long[]> addIntegerArray(
            String name,
            Supplier<long[]> valueSupplier,
            long[] defaultValue) {
        IntegerArrayField field = new IntegerArrayField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<long[]> addIntegerArray(String name, Supplier<long[]> valueSupplier) {
        return addIntegerArray(name, valueSupplier, new long[] {});
    }

    public Supplier<float[]> addFloatArray(
            String name,
            Supplier<float[]> valueSupplier,
            float[] defaultValue) {
        FloatArrayField field = new FloatArrayField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<float[]> addFloatArray(String name, Supplier<float[]> valueSupplier) {
        return addFloatArray(name, valueSupplier, new float[] {});
    }

    public Supplier<double[]> addDoubleArray(
            String name,
            Supplier<double[]> valueSupplier,
            double[] defaultValue) {
        DoubleArrayField field = new DoubleArrayField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
    }

    public Supplier<double[]> addDoubleArray(String name, Supplier<double[]> valueSupplier) {
        return addDoubleArray(name, valueSupplier, new double[] {});
    }

    public Supplier<String[]> addStringArray(
            String name,
            Supplier<String[]> valueSupplier,
            String[] defaultValue) {
        StringArrayField field = new StringArrayField(name, valueSupplier, defaultValue);
        fields.add(field);
        return field;
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