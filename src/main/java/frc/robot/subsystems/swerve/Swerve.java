package frc.robot.subsystems.swerve;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.system.Timer;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.LinearVelocity;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.poseestimation.PoseEstimator.OdometryMeasurement;
import frc.robot.subsystems.swerve.SwerveConstants.Modules;
import frc.robot.subsystems.swerve.io.ImuIO;
import frc.robot.subsystems.swerve.io.ImuIORedux;
import frc.robot.subsystems.swerve.io.ImuIOSim;
import frc.robot.utils.RotationalSensorHelper;
import frc.robot.utils.TunableCommand;
import frc.robot.utils.TunableMeasure;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Seconds;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.getModuleName;
import static frc.robot.Constants.ROBOT_PERIOD;
import static frc.robot.RobotMap.CANBUS;

import java.util.Optional;

public class Swerve implements Mechanism, ComplexTunable {
    private LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private SwerveModule[] modules = {
        new SwerveModule(fieldsTable, CANBUS.MODULE_FL, 0),
        new SwerveModule(fieldsTable, CANBUS.MODULE_FR, 1),
        new SwerveModule(fieldsTable, CANBUS.MODULE_BL, 2),
        new SwerveModule(fieldsTable, CANBUS.MODULE_BR, 3),
    };

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULES_LOCATIONS);

    private final RotationalSensorHelper gyroYaw;

    private final ImuIO imuIO = Robot.isReal() ? new ImuIORedux(fieldsTable) : new ImuIOSim(fieldsTable);

    private final Debouncer isGyroConnectedDebouncer = new Debouncer(GYRO_CONNECTED_DEBOUNCE_TIME.in(Seconds), DebounceType.FALLING);

    public Swerve() {
        fieldsTable.update();

        gyroYaw = new RotationalSensorHelper(imuIO.yawAngle.get());
        gyroYaw.enableContinuousWrap(GYRO_ANGLE_BOUNDS);

        Scheduler.getDefault().addPeriodic(this::periodic);

        PeriodicAlertsGroup.defaultInstance.addMediumAlert(() -> "Gyro Disconnected!", () -> !isGyroConnected());
    }

    public void periodic() {
        for (SwerveModule module : modules) {
            module.periodic();
        }

        gyroYaw.update(imuIO.yawAngle.get());

        Optional<Rotation2d> gyroAngle = isGyroConnected() ? Optional.of(new Rotation2d(getGyroYaw()))
            : Optional.empty();
        PoseEstimator.getInstance().addOdometryMeasurement(new OdometryMeasurement(
                Seconds.of(Timer.getTimestamp()),
                kinematics,
                getModulePositionsDelta(),
                getModulesVelocities(),
                gyroAngle));

        fieldsTable.recordOutputMeasure("Yaw", getGyroYaw());
        fieldsTable.recordOutput("Is gyro connected", isGyroConnected());
        fieldsTable.recordOutput("Robot Relative Real Chassis Velocities", getRobotRelativeChassisVelocities());
        fieldsTable.recordOutput("Robot Relative Real Velocities", getModulesVelocities());
        fieldsTable.recordOutput("Modules Current Positions", getModulePositions());
        fieldsTable.recordOutput("Current Commands", getRunningCommands().stream().map(c -> c.name()).toArray(String[]::new));
    }

    public void drive(LinearVelocity vx, LinearVelocity vy, AngularVelocity vtheta, boolean isFieldRelative,
        boolean useVoltage) {
        fieldsTable.recordOutputMeasure("Desired vx", vx);
        fieldsTable.recordOutputMeasure("Desired vy", vy);
        fieldsTable.recordOutputMeasure("Desired vtheta", vtheta);
        fieldsTable.recordOutput("isFieldRelativeDrive", isFieldRelative);
        fieldsTable.recordOutput("voltageDrive", useVoltage);

        ChassisVelocities targetChassisVelocities;
        if (isFieldRelative) {
            boolean isRed = RobotContainer.isRedAlliance();
            ChassisVelocities targetChassisVeloctiesFieldRelative = new ChassisVelocities(isRed ? vx.unaryMinus() : vx, isRed ? vy.unaryMinus() : vy, vtheta);
            targetChassisVelocities = targetChassisVeloctiesFieldRelative.toRobotRelative(PoseEstimator.getInstance().getEstimatedPose().getRotation());
        } else {
            targetChassisVelocities = new ChassisVelocities(vx, vy, vtheta);
        }

        driveChassisVelocities(targetChassisVelocities, useVoltage);
    }

    public void stop() {
        drive(MetersPerSecond.zero(), MetersPerSecond.zero(), DegreesPerSecond.zero(), false, true);
    }

    public void driveChassisVelocities(ChassisVelocities robotRelativeChassisVelocities, boolean useVoltage) {
        robotRelativeChassisVelocities = robotRelativeChassisVelocities.discretize(ROBOT_PERIOD.in(Seconds));

        fieldsTable.recordOutput("Robot Relative Target ChassisVelocities", robotRelativeChassisVelocities);

        SwerveModuleVelocity[] moduleVelocitys = kinematics.toSwerveModuleVelocities(robotRelativeChassisVelocities);

        moduleVelocitys = SwerveDriveKinematics.desaturateWheelVelocities(moduleVelocitys, Modules.MAX_DRIVE_VELOCITY);

        setModulesVelocities(moduleVelocitys, true, true, useVoltage);
    }

    public void setModulesVelocities(SwerveModuleVelocity[] moduleVelocitys, boolean optimize, boolean preventJittering,
        boolean useVoltage) {
        fieldsTable.recordOutput("Modules Target Velocitys", moduleVelocitys);

        for (SwerveModule module : modules)
            module.setTargetVelocity(moduleVelocitys[module.getModuleNum()], optimize, preventJittering, useVoltage);
    }

    public void runDriveCharacterization(Current current) {
        for (SwerveModule module : modules) {
            module.runCharacterization(current);
        }
    }

    public void setGyroOffset(Angle newGyroOffset) {
        gyroYaw.setOffset(newGyroOffset);
    }

    public void coastAll() {
        for (SwerveModule module : modules) {
            module.setCoast();
        }
    }

    public Angle getGyroYaw() {
        return gyroYaw.getAngle();
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    public SwerveModulePosition[] getModulePositionsDelta() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePositionDelta();
        }
        return positions;
    }

    public SwerveModuleVelocity[] getModulesVelocities() {
        SwerveModuleVelocity[] moduleVelocities = new SwerveModuleVelocity[modules.length];
        for (int i = 0; i < moduleVelocities.length; i++) {
            moduleVelocities[i] = modules[i].getModuleVelocity();            
        }
        return moduleVelocities;
    }

    public ChassisVelocities getRobotRelativeChassisVelocities() {
        return kinematics.toChassisVelocities(getModulesVelocities());
    }

    public ChassisVelocities getFieldRelativeChassisVelocities() {
        return getRobotRelativeChassisVelocities().toFieldRelative(PoseEstimator.getInstance().getEstimatedPose().getRotation());
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public boolean isGyroConnected() {
        return isGyroConnectedDebouncer.calculate(imuIO.isConnected.getAsBoolean());
    }

    @Override
    public void publishTunable(TunableTable table) {
        for (SwerveModule module : modules) {
            int moduleNum = module.getModuleNum();
            table.publish("Module " + moduleNum + " " + getModuleName(moduleNum), module);
        }

        table.publish("Yaw RotationalSensorHelper", gyroYaw);

        table.publish("Reset Modules Angle", (resetTable) -> {
            TunableMeasure<Angle> angleToReset = new TunableMeasure<Angle>(Degrees.zero(), Degrees);
            resetTable.publish("Angle to reset", angleToReset.tunable());
            resetTable.publish("Reset", new TunableCommand(Command.noRequirements((coroutine) -> {
                Angle newAngle = angleToReset.get();
                for (SwerveModule module : modules)
                    module.resetAngle(newAngle);
            }).named("Reset ")));
        });
    }
}
