package frc.robot.subsystems.swerve.io;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

import frc.robot.RobotMap.CANBUS.ModuleCanAddresses;
import frc.robot.utils.AlertsFactory;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class SwerveModuleIOTalonFX extends SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;

    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<Current> driveTorqueCurrent;
    private final StatusSignal<Current> turnTorqueCurrent;
    
    private final VoltageOut driveVoltageControl = new VoltageOut(0);
    private final VelocityTorqueCurrentFOC driveVelocityControl = new VelocityTorqueCurrentFOC(0).withSlot(0);
    private final TorqueCurrentFOC driveCurrentControl = new TorqueCurrentFOC(0);

    private final PositionVoltage turnVoltageControl = new PositionVoltage(0).withSlot(0);

    private StatusCode driveMotorStatus;
    private StatusCode turnMotorStatus;
    private StatusCode canCoderStatus;

    private final Slot0Configs turnSlotConfigs;

    public SwerveModuleIOTalonFX(LogFieldsTable fieldsTable, ModuleCanAddresses moduleCanAddresses, int moduleNum) {
        super(fieldsTable);
        driveMotor = new TalonFX(moduleCanAddresses.drive().id(), new CANBus(moduleCanAddresses.drive().port()));
        turnMotor = new TalonFX(moduleCanAddresses.turn().id(), new CANBus(moduleCanAddresses.turn().port()));
        canCoder = new CANcoder(moduleCanAddresses.canCoder().id(), new CANBus(moduleCanAddresses.canCoder().port()));

        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();

        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        driveMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT.in(Amps);

        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT.in(Amps);
        driveMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = DRIVE_SUPPLY_CURRENT_LOWER_LIMIT.in(Amps);
        driveMotorConfig.CurrentLimits.SupplyCurrentLowerTime = DRIVE_SUPPLY_CURRENT_LOWER_TIME.in(Seconds);

        Slot0Configs driveSlotConfig = driveMotorConfig.Slot0;
        driveSlotConfig.kS = DRIVE_MOTOR_KS.in(Amps);
        driveSlotConfig.kV = DRIVE_MOTOR_KV.in(Amps.per(MetersPerSecond)) * WHEEL_CIRCUMFERENCE.in(Meters);
        driveSlotConfig.kA = DRIVE_MOTOR_KA.in(Amps.per(MetersPerSecondPerSecond)) * WHEEL_CIRCUMFERENCE.in(Meters);;

        driveSlotConfig.kP = DRIVE_MOTOR_KP;
        driveSlotConfig.kI = DRIVE_MOTOR_KI;
        driveSlotConfig.kD = DRIVE_MOTOR_KD;

        TalonFXConfiguration turnMotorConfig = new TalonFXConfiguration();

        turnMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        turnMotorConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        turnMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnMotorConfig.Feedback.RotorToSensorRatio = TURN_GEAR_RATIO;
        turnMotorConfig.Feedback.SensorToMechanismRatio = 1.0;

        turnMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;

        turnMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnMotorConfig.CurrentLimits.StatorCurrentLimit = TURN_STATOR_CURRENT_LIMIT.in(Amps);

        turnSlotConfigs = turnMotorConfig.Slot0;
        turnSlotConfigs.kP = TURN_MOTOR_KP;
        turnSlotConfigs.kI = TURN_MOTOR_KI;
        turnSlotConfigs.kD = TURN_MOTOR_KD;

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        driveMotorStatus = driveMotor.getConfigurator().apply(driveMotorConfig);
        turnMotorStatus = turnMotor.getConfigurator().apply(turnMotorConfig);
        canCoderStatus = canCoder.getConfigurator().apply(canCoderConfig);
        
        driveMotor.setPosition(0);

        String moduleAlertPrefix = "Module " + moduleNum + " " + getModuleName(moduleNum) + " ";

        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance,
                () -> driveMotorStatus, moduleAlertPrefix + "Drive Motor Status");
        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance,
                () -> turnMotorStatus, moduleAlertPrefix + "Turn Motor Status");
        AlertsFactory.phoenixMotor(PeriodicAlertsGroup.defaultInstance,
                () -> canCoderStatus, moduleAlertPrefix + "Can Coder Status");

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        turnPosition = turnMotor.getPosition();
        driveTorqueCurrent = driveMotor.getTorqueCurrent();
        turnTorqueCurrent = turnMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(250.0, drivePosition, driveVelocity, turnPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveTorqueCurrent, turnTorqueCurrent);
        driveMotor.optimizeBusUtilization();
        turnMotor.optimizeBusUtilization();
        canCoder.optimizeBusUtilization();
    }

    @Override
    protected void periodicBeforeFields() {
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, turnPosition, driveTorqueCurrent, turnTorqueCurrent);
    }

    @Override
    public void setCoast() {
        driveMotor.setControl(new CoastOut());
        turnMotor.setControl(new CoastOut());
    }

    @Override
    protected Distance getDriveDistance() {
        return WHEEL_CIRCUMFERENCE.times(drivePosition.getValue().in(Rotations));
    }

    @Override
    protected LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(
            driveVelocity.getValue().in(RotationsPerSecond) * WHEEL_CIRCUMFERENCE.in(Meters)
        );
    }

    @Override
    protected Angle getTurnAngle() {
        return turnPosition.getValue();
    }

    @Override
    protected Current getDriveCurrent() {
        return driveTorqueCurrent.getValue();
    }

    @Override
    protected Current getTurnCurrent() {
        return turnTorqueCurrent.getValue();
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        driveMotor.setControl(driveVoltageControl.withOutput(voltage));
    }

    @Override
    public void setDriveVelocity(LinearVelocity velocity) {
        driveMotor.setControl(driveVelocityControl.withVelocity(velocity.in(MetersPerSecond) / WHEEL_CIRCUMFERENCE.in(Meters)));
    }

    @Override
    public void setDriveCurrent(Current current) {
        driveMotor.setControl(driveCurrentControl.withOutput(current));
    }

    @Override
    public void setDesiredAngle(Angle angle) {
        turnMotor.setControl(turnVoltageControl.withPosition(angle));
    }
}
