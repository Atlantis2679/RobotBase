package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;

import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import frc.robot.Robot;
import frc.robot.RobotMap.CANBUS.ModuleCanAddresses;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOTalonFX;
import frc.robot.subsystems.swerve.io.SwerveModuleSim;
import frc.robot.utils.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class SwerveModule implements ComplexTunable {
    private final LogFieldsTable fieldsTable;
    private final SwerveModuleIO io;
    private final int moduleNum;
    private final RotationalSensorHelper rotationalSensorHelper;

    private Distance lastDriveDistance;
    private Distance currentDriveDistance;

    public SwerveModule(LogFieldsTable swerveFieldsTable, ModuleCanAddresses moduleCanAddresses, int moduleNum) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moduleNum + " " + getModuleName(moduleNum));
        io = Robot.isReal() ? new SwerveModuleIOTalonFX(fieldsTable, moduleCanAddresses, moduleNum)
                : new SwerveModuleSim(fieldsTable, moduleCanAddresses, moduleNum);

        fieldsTable.update();

        this.moduleNum = moduleNum;

        rotationalSensorHelper = new RotationalSensorHelper(io.turnAngle.get(), OFFSETS[moduleNum]);
        rotationalSensorHelper.enableContinuousWrap(TURN_ANGLE_BOUNDS);

        currentDriveDistance = getDriveDistance();
    }

    public void periodic() {
        rotationalSensorHelper.update(io.turnAngle.get());

        lastDriveDistance = currentDriveDistance;
        currentDriveDistance = getDriveDistance();

        fieldsTable.recordOutputMeasure("Angle", getAngle());
        fieldsTable.recordOutput("Module Position", getModulePosition());
        fieldsTable.recordOutput("Module Position Delta", getModulePositionDelta());
    }

    public void setTargetVelocity(SwerveModuleVelocity targetVelocity, boolean optimize, boolean preventJittering,
            boolean useVoltage) {
        Rotation2d currentAngle = new Rotation2d(getAngle());
        if (optimize) targetVelocity = targetVelocity.optimize(currentAngle).cosineScale(currentAngle);

        LinearVelocity targetStateVelocity = MetersPerSecond.of(targetVelocity.velocity);

        if (preventJittering
                && targetStateVelocity.abs(MetersPerSecond) < MAX_DRIVE_VELOCITY.times(PREVENT_JITTERING_MULTIPLIER).abs(MetersPerSecond)) {
            if (useVoltage) io.setDriveVoltage(Volts.zero());
            else io.setDriveVelocity(MetersPerSecond.zero());
            return;
        }

        fieldsTable.recordOutput("Module target state", targetVelocity);
        if (useVoltage) {
            fieldsTable.recordOutputMeasure("Drive motor target voltage", targetStateVelocity.div(MAX_DRIVE_VELOCITY).times(MAX_VOLTAGE));
            io.setDriveVoltage(targetStateVelocity.div(MAX_DRIVE_VELOCITY).times(MAX_VOLTAGE));
        } else {
            fieldsTable.recordOutput("Drive motor target velocity", targetStateVelocity);
            io.setDriveVelocity(targetStateVelocity);
        }

        fieldsTable.recordOutputMeasure("Turn motor target angle", targetVelocity.angle.getMeasure());
        io.setDesiredAngle(targetVelocity.angle.getMeasure().plus(rotationalSensorHelper.getOffset())); // Converts back to not offseted angle
    }

    public void runCharacterization(Current current) {
        io.setDesiredAngle(rotationalSensorHelper.getOffset()); // Point wheel forwards
        io.setDriveCurrent(current);
    }

    public void resetAngle(Angle newAngle) {
        rotationalSensorHelper.resetAngle(newAngle);
    }

    public void setCoast() {
        io.setCoast();
    }

    public Angle getAngle() {
        return rotationalSensorHelper.getAngle();
    }

    public int getModuleNum() {
        return moduleNum;
    }

    public Distance getDriveDistance() {
        return io.driveDistance.get();
    }

    public Current getDriveCurrent() {
        return io.driveCurrent.get();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(currentDriveDistance, new Rotation2d(getAngle()));
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(currentDriveDistance.minus(lastDriveDistance), new Rotation2d(getAngle())); // The angle doesn't use the delta
    }

    public SwerveModuleVelocity getModuleVelocity() {
        return new SwerveModuleVelocity(getDriveVelocity(), new Rotation2d(getAngle()));
    }

    public LinearVelocity getDriveVelocity() {
        return io.driveVelocity.get();
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publish("RotationalSensorHelper", rotationalSensorHelper);
    }
}
