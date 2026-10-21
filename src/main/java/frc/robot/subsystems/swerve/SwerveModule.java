package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;

import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import frc.robot.Robot;
import frc.robot.RobotMap.CANBUS.ModuleCanAddresses;
import frc.robot.subsystems.swerve.io.SwerveModuleIO;
import frc.robot.subsystems.swerve.io.SwerveModuleIOTalonFX;
import frc.robot.subsystems.swerve.io.SwerveModuleSim;
import frc.robot.utils.TunableMeasure;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class SwerveModule implements ComplexTunable {
    private final LogFieldsTable fieldsTable;
    private final SwerveModuleIO io;
    private final int moduleNum;

    private Distance lastDriveDistance;
    private Distance currentDriveDistance;
    private Angle lastAngle;
    private Angle currentAngle;

    private PIDController pidController;

    public SwerveModule(LogFieldsTable swerveFieldsTable, ModuleCanAddresses moduleCanAddresses, int moduleNum) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moduleNum + " " + getModuleName(moduleNum));
        io = Robot.isReal() ? new SwerveModuleIOTalonFX(fieldsTable, moduleCanAddresses, moduleNum)
                : new SwerveModuleSim(fieldsTable, moduleCanAddresses, moduleNum);

        fieldsTable.update();

        this.moduleNum = moduleNum;
    }

    public void periodic() {
        lastDriveDistance = currentDriveDistance;
        currentDriveDistance = getDriveDistance();
        lastAngle = currentAngle;
        currentAngle = getAngle();

        fieldsTable.recordOutputMeasure("Angle", currentAngle);
        fieldsTable.recordOutputMeasure("Drive Distance", currentDriveDistance);
        fieldsTable.recordOutput("Module Position", getModulePosition());
        fieldsTable.recordOutput("Module Position Delta", getModulePositionDelta());
    }

    public void setTargetVelocity(SwerveModuleVelocity targetVelocity, boolean optimize, boolean preventJittering,
            boolean useVoltage) {
        Rotation2d currentAngle = new Rotation2d(this.currentAngle);
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
        io.setDesiredAngle(targetVelocity.angle.getMeasure());
    }

    public void resetAngle(Angle newAngle) {
        io.resetAngle(newAngle);
    }

    public void setCoast() {
        io.setCoast();
    }

    public void setTurnPID(PIDController pidController) {
        this.pidController = pidController;
        io.setTurnPID(pidController);
    }

    public Angle getAngle() {
        return io.turnAngle.get();
    }

    public int getModuleNum() {
        return moduleNum;
    }

    public Distance getDriveDistance() {
        return io.driveDistance.get();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getAngle()));
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(getDriveDistance().minus(lastDriveDistance), new Rotation2d(getAngle().minus(lastAngle)));
    }

    public SwerveModuleVelocity getModuleVelocity() {
        return new SwerveModuleVelocity(getDriveVelocity(), new Rotation2d(getAngle()));
    }

    public PIDController getTurnPID() {
        return pidController;
    }

    public Current getCurrent() {
        return io.driveCurrent.get();
    }
    
    public LinearVelocity getDriveVelocity() {
        return io.driveVelocity.get();
    }

    @Override
    public void publishTunable(TunableTable table) {
        TunableMeasure<Angle> angleToReset = TunableMeasure.of(Degrees.zero(), Degrees);
        table.publish("Angle To Reset", angleToReset.tunable());
    }
}
