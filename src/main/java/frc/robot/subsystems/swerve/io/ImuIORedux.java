package frc.robot.subsystems.swerve.io;


import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.RobotMap.CANBUS.CAN_AND_GYRO_ID;
import static org.wpilib.units.Units.Gs;
import static org.wpilib.units.Units.Rotations;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.LinearAcceleration;

import com.reduxrobotics.canandgyro.Canandgyro;

public class ImuIORedux extends ImuIO {
    private final Canandgyro canandgyro = new Canandgyro(CAN_AND_GYRO_ID.id(), CAN_AND_GYRO_ID.port());

    public ImuIORedux(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    @Override
    protected Angle getYaw() {
        return Rotations.of(canandgyro.getYaw());
    }

    @Override
    protected Angle getPitch() {
        return Rotations.of(canandgyro.getPitch());
    }

    @Override
    protected Angle getRoll() {
        return Rotations.of(canandgyro.getRoll());
    }

    @Override
    protected boolean getIsConnected() {
        return canandgyro.isConnected();
    }

    @Override
    protected LinearAcceleration getXAcceleration() {
        return Gs.of(canandgyro.getAccelerationX());
    }

    @Override
    protected LinearAcceleration getYAcceleration() {
        return Gs.of(canandgyro.getAccelerationY());
    }

    @Override
    protected LinearAcceleration getZAcceleration() {
        return Gs.of(canandgyro.getAccelerationZ());
    }
}
