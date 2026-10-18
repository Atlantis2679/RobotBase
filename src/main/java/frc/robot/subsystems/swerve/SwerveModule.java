package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;

import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.math.controller.PIDController;
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
    private final RotationalSensorHelper absoluteAngleDegreesCWW;
    private final int moduleNum;

    private Distance lastDriveDistance;
    private Distance currentDriveDistance;

    private Angle currentIntegratedAngle;

    private PIDController pidController;

    public SwerveModule(LogFieldsTable swerveFieldsTable, ModuleCanAddresses moduleCanAddresses, int moduleNum) {
        fieldsTable = swerveFieldsTable.getSubTable("Module " + moduleNum + " " + getModuleName(moduleNum));
        io = Robot.isReal() ? new SwerveModuleIOTalonFX(fieldsTable, moduleCanAddresses, moduleNum)
                : new SwerveModuleSim(fieldsTable);

        fieldsTable.update();

        this.moduleNum = moduleNum;

        absoluteAngleDegreesCWW = new RotationalSensorHelper(io.absoluteTurnAngle.get().in(Degrees), OFFSETS[moduleNum]);
        absoluteAngleDegreesCWW.enableContinuousWrap(ANGLE_BOUNDS);

        resetIntegratedAngleToAbsolute();
    }

    public void periodic() {
        absoluteAngleDegreesCWW.update(io.absoluteTurnAngle.get().in(Degrees));
        lastDriveDistance = currentDriveDistance;
        currentDriveDistance = getDriveDistance();

        currentIntegratedAngle = getIntegratedAngle();

        fieldsTable.recordOutput("Absolute Angle", getAbsoluteDegreesCCW());
        fieldsTable.recordOutput("Integrated Angles", getIntegratedAngle());
        fieldsTable.recordOutput("Drive Distance", getDriveDistanceMeters());
        fieldsTable.recordOutput("Module Position", getModulePosition());
        fieldsTable.recordOutput("Module Position Delta", getModulePositionDelta());
    }

    public void setTargetState(SwerveModuleVelocity targetState, boolean optimize, boolean preventJittering,
            boolean useVoltage) {
        if (preventJittering
                && Math.abs(targetState.velocity) < MAX_SPEED_MPS * PREVENT_JITTERING_MULTIPLIER) {
            io.setDrivePercentageSpeed(0);
            return;
        }

        if (optimize)
            targetState.optimize(Rotation2d.fromDegrees(currentAngleDegreesCCW));

        fieldsTable.recordOutput("Module target state", targetState);
        if (useVoltage) {
            fieldsTable.recordOutput("Drive motor target voltage", (targetState.velocity / MAX_SPEED_MPS) * MAX_VOLTAGE);
            io.setDriveVoltage((targetState.velocity / MAX_SPEED_MPS) * MAX_VOLTAGE);
        } else {
            fieldsTable.recordOutput("Drive motor target speed", targetState.velocity / MAX_SPEED_MPS);
            io.setDrivePercentageSpeed(targetState.velocity / MAX_SPEED_MPS);
        }

        fieldsTable.recordOutput("Turn motor target rotation", targetState.angle.getRotations());
        io.setTurnAngleRotations(targetState.angle.getRotations());
    }

    public double getAbsoluteDegreesCCW() {
        return absoluteAngleDegreesCWW.getAngle();
    }

    public int getModuleNum() {
        return moduleNum;
    }

    public Distance getDriveDistance() {
        return io.driveDistanceRotations.get();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getIntegratedAngle()));
    }

    public SwerveModulePosition getModulePositionDelta() {
        return new SwerveModulePosition(getDriveDistance().minus(lastDriveDistance),
                Rotation2d.fromDegrees(getIntegratedDegreesCCW()));
    }

    public SwerveModuleVelocity getModuleState() {
        return new SwerveModuleVelocity(getVelocityMPS(), Rotation2d.fromDegrees(getIntegratedDegreesCCW()));
    }
    
    public LinearVelocity getDriveVelocity() {
        return io.driveVelocity.get();
    }

    public Angle getIntegratedAngle() {
        return io.integratedTurnAngle.get();
    }

    public void resetIntegratedAngleToAbsolute() {
        currentAngleDegreesCCW = getAbsoluteDegreesCCW();
        io.resetIntegratedAngleRotations(currentAngleDegreesCCW / 360);
    }

    public void resetAngleDegreesCCW(double newAngle) {
        absoluteAngleDegreesCWW.resetAngle(newAngle);
        resetIntegratedAngleToAbsolute();
    }

    public void setCoast() {
        io.setCoast();
    }

    public void setTurnPID(PIDController pidController) {
        this.pidController = pidController;
        io.setTurnPID(pidController);
    }

    public PIDController getTurnPID() {
        return pidController;
    }

    public Current getCurrent() {
        return io.driveCurrent.get();
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publishDouble("Integrated Angle Degrees CCW", this::getIntegratedDegreesCCW, null);
        table.publishDouble("Absolute Angle Degrees CCW", this::getAbsoluteDegreesCCW, null);
        table.publishDouble("Tunable Offset", absoluteAngleDegreesCWW::getOffset, newOffset -> {
            absoluteAngleDegreesCWW.setOffset(newOffset);
            resetIntegratedAngleToAbsolute();
        });
    }
}
