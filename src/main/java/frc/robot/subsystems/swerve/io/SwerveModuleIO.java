package frc.robot.subsystems.swerve.io;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import java.util.function.Supplier;

import org.wpilib.math.controller.PIDController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class SwerveModuleIO extends IOBase {
    public final Supplier<Angle> turnAngle = fields.addMeasure("turnAngle",
        this::getTurnAngle, Degrees.zero());
    public final Supplier<Distance> driveDistance = fields.addMeasure("driveDistance",
        this::getDriveDistance, Meters.zero());
    public final Supplier<LinearVelocity> driveVelocity = fields.addMeasure("driveVelocity",
        this::getDriveVelocity, MetersPerSecond.zero());
    public final Supplier<Current> driveCurrent = fields.addMeasure("driveCurrent",
        this::getDriveCurrent, Amps.zero());
    public final Supplier<Current> turnCurrent = fields.addMeasure("driveCurrent",
        this::getDriveCurrent, Amps.zero());

    public SwerveModuleIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract Distance getDriveDistance();
    protected abstract LinearVelocity getDriveVelocity();

    protected abstract Angle getTurnAngle();

    protected abstract Current getDriveCurrent();
    protected abstract Current getTurnCurrent();

    public abstract void setDriveVoltage(Voltage voltage);
    public abstract void setDriveVelocity(LinearVelocity speed);

    public abstract void setDesiredAngle(Angle angle);

    public abstract void setCoast();

    public abstract void setTurnPID(PIDController pidController);
}
