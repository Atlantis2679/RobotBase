package frc.robot.subsystems.swerve.io;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.LinearAcceleration;

import team2679.atlantiskit.logfields.LogFieldsTable;

public class ImuIOSim extends ImuIO {
    public ImuIOSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected Angle getYaw() {
        return Degrees.zero();
    }

    @Override
    protected Angle getPitch() {
        return Degrees.zero();
    }

    @Override
    protected Angle getRoll() {
        return Degrees.zero();
    }

    @Override
    protected boolean getIsConnected() {
        return false;
    }

    @Override
    protected LinearAcceleration getXAcceleration() {
        return MetersPerSecondPerSecond.zero();
    }

    @Override
    protected LinearAcceleration getYAcceleration() {
        return MetersPerSecondPerSecond.zero();
    }

    @Override
    protected LinearAcceleration getZAcceleration() {
        return MetersPerSecondPerSecond.zero();
    }
}
