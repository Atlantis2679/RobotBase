package frc.robot.subsystems.swerve.io;

import java.util.function.Supplier;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.LinearAcceleration;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;

import java.util.function.BooleanSupplier;

public abstract class ImuIO extends IOBase {
    public final Supplier<Angle> yawAngle = fields.addMeasure("yawAngle", this::getYaw, Degrees.zero());
    public final Supplier<Angle> pitchAngle = fields.addMeasure("pitchAngle", this::getPitch, Degrees.zero());
    public final Supplier<Angle> rollAngle = fields.addMeasure("rollAngle", this::getRoll, Degrees.zero());

    public final BooleanSupplier isConnected = fields.addBoolean("isConnected", this::getIsConnected);

    public final Supplier<LinearAcceleration> xAcceleration = fields.addMeasure("X Acceleration", this::getXAcceleration, MetersPerSecondPerSecond.zero());
    public final Supplier<LinearAcceleration> yAcceleration = fields.addMeasure("Y Acceleration", this::getYAcceleration, MetersPerSecondPerSecond.zero());
    public final Supplier<LinearAcceleration> zAcceleration = fields.addMeasure("Z Acceleration", this::getZAcceleration, MetersPerSecondPerSecond.zero());

    public ImuIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract Angle getYaw();
    protected abstract Angle getPitch();
    protected abstract Angle getRoll();

    protected abstract boolean getIsConnected();

    protected abstract LinearAcceleration getXAcceleration();
    protected abstract LinearAcceleration getYAcceleration();
    protected abstract LinearAcceleration getZAcceleration();
}
