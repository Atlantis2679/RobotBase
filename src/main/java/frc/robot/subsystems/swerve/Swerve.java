package frc.robot.subsystems.swerve;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableTable;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap.ModuleFL;
import frc.robot.RobotMap.ModuleFR;
import frc.robot.RobotMap.ModuleBL;
import frc.robot.RobotMap.ModuleBR;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.poseestimation.PoseEstimator.OdometryMeasurement;
import frc.robot.subsystems.swerve.SwerveConstants.Modules;
import frc.robot.subsystems.swerve.io.ImuIO;
import frc.robot.subsystems.swerve.io.ImuIORedux;
import frc.robot.subsystems.swerve.io.ImuIOSim;
import frc.robot.utils.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

import static org.wpilib.units.Units.Seconds;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.getModuleName;

import java.util.Optional;
import java.util.Timer;

public class Swerve implements Mechanism, ComplexTunable {
  private LogFieldsTable fieldsTable = new LogFieldsTable(getName());

  private SwerveModule[] modules = {
      new SwerveModule(fieldsTable, 0, ModuleFL.DRIVE_MOTOR_ID, ModuleFL.TURN_MOTOR_ID, ModuleFL.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 1, ModuleFR.DRIVE_MOTOR_ID, ModuleFR.TURN_MOTOR_ID, ModuleFR.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 2, ModuleBL.DRIVE_MOTOR_ID, ModuleBL.TURN_MOTOR_ID, ModuleBL.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 3, ModuleBR.DRIVE_MOTOR_ID, ModuleBR.TURN_MOTOR_ID, ModuleBR.CAN_CODER_ID),
  };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULES_LOCATIONS);

  private final RotationalSensorHelper gyroYawDegreesCCW;

  private final ImuIO imuIO = Robot.isReal() ? new ImuIORedux(fieldsTable) : new ImuIOSim(fieldsTable);

  private final Debouncer isGyroConnectedDebouncer = new Debouncer(GYRO_CONNECTED_DEBUNCER_SECONDS, DebounceType.kFalling);

  public Swerve() {
    fieldsTable.update();

    TunablesManager.add("Swerve", (Tunable) this);

    gyroYawDegreesCCW = new RotationalSensorHelper(imuIO.yawAngleDeg.getAsDouble());
    gyroYawDegreesCCW.enableContinuousWrap(0, 360);

    PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Gyro Disconnected!", () -> !isGyroConnected());
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }

    gyroYawDegreesCCW.update(imuIO.yawAngleDeg.getAsDouble());

    Optional<Rotation2d> gyroAngle = isGyroConnected() ? Optional.of(Rotation2d.fromDegrees(getGyroYawDegreesCCW()))
        : Optional.empty();
    PoseEstimator.getInstance().addOdometryMeasurement(
        new OdometryMeasurement(
          Seconds.of(Timer.getTimestamp()),
          kinematics,
          getModulePositionsDelta(),
          getModulesVelocities(),
          new Translation2d(imuIO.xAcceleration.getAsDouble(), imuIO.yAcceleration.getAsDouble()),
          gyroAngle
    ));

    fieldsTable.recordOutput("Is gryo connected", isGyroConnected());
    fieldsTable.recordOutput("Robot Relative Real Chassis Velocities", getRobotRelativeChassisVelocities());
    fieldsTable.recordOutput("Robot Relative Real Velocities", getModulesVelocities());
    fieldsTable.recordOutput("Yaw degrees CCW", getGyroYawDegreesCCW());
    fieldsTable.recordOutput("Modules Current Positions", getModulePositions());
    fieldsTable.recordOutput("Current Commands", getRunningCommands());
  }

  public void drive(double vxSpeedMPS, double vySpeedMPS, double vAngleRandiansPS, boolean isFieldRelative,
      boolean useVoltage) {
    fieldsTable.recordOutput("vxSpeedMPS", vxSpeedMPS);
    fieldsTable.recordOutput("vySpeedMPS", vySpeedMPS);
    fieldsTable.recordOutput("isFieldRelative", isFieldRelative);
    ChassisVelocities targetChassisVelocities = isFieldRelative ? ChassisVelocities.fromFieldRelativeVelocities(
        RobotContainer.isRedAlliance() ? -vxSpeedMPS : vxSpeedMPS,
        RobotContainer.isRedAlliance() ? -vySpeedMPS : vySpeedMPS,
        vAngleRandiansPS, PoseEstimator.getInstance().getEstimatedPose().getRotation())
        : new ChassisVelocities(vxSpeedMPS, vySpeedMPS, vAngleRandiansPS);

    driveChassisVelocities(targetChassisVelocities, useVoltage);
  }

  public void stop() {
    drive(0, 0, 0, false, true);
  }

  public void driveChassisVelocities(ChassisVelocities Velocities, boolean useVoltage) {
    fieldsTable.recordOutput("Modules Target Chassis Velocities", Velocities);

    SwerveModuleVelocity[] moduleVelocitys = kinematics.toSwerveModuleVelocities(Velocities);

    SwerveDriveKinematics.desaturateWheelVelocities(moduleVelocitys, Modules.MAX_SPEED_MPS);

    setModulesVelocities(moduleVelocitys, true, true, useVoltage);
  }

  public void resetModulesToAbsoulte() {
    for (SwerveModule module : modules)
      module.resetIntegratedAngleToAbsolute();
  }

  public void setModulesVelocities(SwerveModuleVelocity[] moduleVelocitys, boolean optimize, boolean preventJittering,
      boolean useVoltage) {
    fieldsTable.recordOutput("Modules Target Velocitys", moduleVelocitys);

    for (SwerveModule module : modules)
      module.setTargetVelocity(moduleVelocitys[module.getModuleNum()], optimize, preventJittering, useVoltage);
  }

  public void setGyroOffset(double newGyroOffsetCCW) {
  }

  public void coastAll() {
    for (SwerveModule module : modules) {
      module.setCoast();
    }
  }

  public double getGyroYawDegreesCCW() {
    return gyroYawDegreesCCW.getAngle();
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

  public ChassisVelocities getRobotRelativeChassisVelocities() {
    return kinematics.toChassisVelocities(
        modules[0].getModuleVelocity(),
        modules[1].getModuleVelocity(),
        modules[2].getModuleVelocity(),
        modules[3].getModuleVelocity());
  }

  public ChassisVelocities getFieldRelativeChassisVelocities() {
    return getRobotRelativeChassisVelocities().toFieldRelative(PoseEstimator.getInstance().getEstimatedPose().getRotation());
  }
  
  public SwerveModuleVelocity[] getModulesVelocities() {
    return new SwerveModuleVelocity[] {
      modules[0].getModuleVelocity(),
      modules[1].getModuleVelocity(),
      modules[2].getModuleVelocity(),
      modules[3].getModuleVelocity()
    };
  }

  public boolean isGyroConnected() {
    return isGyroConnectedDebouncer.calculate(imuIO.isConnected.getAsBoolean());
  }

  public double getXAcceleration() {
    return imuIO.xAcceleration.getAsDouble();
  }

  public double getYAcceleration() {
    return imuIO.yAcceleration.getAsDouble();
  }

  public double getZAcceleration() {
    return imuIO.zAcceleration.getAsDouble();
  }

  public double getRobotPitchDeg() {
    return imuIO.pitchAngleDeg.getAsDouble();
  }

  public double getRobotRollDeg() {
    return imuIO.rollAngleDeg.getAsDouble();
  }

  public double[] getModulesCurrents() {
    double[] currents = new double[4];
    for (int i = 0; i < 4; ++i) {
      currents[i] = Math.abs(modules[i].getCurrent());
    }
    return currents;
  }

  @Override
  public void initTunable(TunableBuilder builder) {
    builder.addChild("Module 0 FL", modules[0]);
    builder.addChild("Module 1 FR", modules[1]);
    builder.addChild("Module 2 BL", modules[2]);
    builder.addChild("Module 3 BR", modules[3]);

    builder.addChild("Reset moudles to absoulte",
        new InstantCommand(this::resetModulesToAbsoulte).ignoringDisable(true));

    builder.addChild("Reset absolute angle", (Tunable) (resetBuilder) -> {
      DoubleHolder angleToReset = new DoubleHolder(0);
      resetBuilder.addDoubleProperty("Angle to reset", angleToReset::get, angleToReset::set);
      resetBuilder.addChild("reset!", new InstantCommand(() -> {
        for (SwerveModule module : modules) {
          module.resetAngleDegreesCCW(angleToReset.get());
        }
      }).ignoringDisable(true));
    });
  }

  @Override
  public void publishTunable(TunableTable table) {
    for (SwerveModule module : modules) {
      int moduleNum = module.getModuleNum();
      table.publish("Module " + moduleNum + " " + getModuleName(moduleNum), module);
    }

    table.publish("Reset absolute angle", (resetTable) -> {
      TunableDouble angleToReset = TunableDouble.create(0);
      resetTable.publish("Angle to reset", angleToReset);
      resetTable.publish("Reset", Command.noRequirements((coroutine) -> {

      }));
    });
  }
}
