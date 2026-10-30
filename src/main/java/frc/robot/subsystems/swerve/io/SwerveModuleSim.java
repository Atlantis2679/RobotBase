package frc.robot.subsystems.swerve.io;

import static frc.robot.Constants.ROBOT_PERIOD;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.DRIVE_MOMENT;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.DRIVE_MOTOR_GEARBOX;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.TURN_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.TURN_MOMENT;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.TURN_MOTOR_GEARBOX;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.RobotController;

import frc.robot.RobotMap.CANBUS.ModuleCanAddresses;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class SwerveModuleSim extends SwerveModuleIOTalonFX {
    private final DCMotorSim driveSim = new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(DRIVE_MOTOR_GEARBOX, DRIVE_MOMENT.in(KilogramSquareMeters), DRIVE_GEAR_RATIO), DRIVE_MOTOR_GEARBOX);
    private final DCMotorSim turnSim = new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(TURN_MOTOR_GEARBOX, TURN_MOMENT.in(KilogramSquareMeters), TURN_GEAR_RATIO), TURN_MOTOR_GEARBOX);

    private final TalonFXSimState driveSimState;
    private final TalonFXSimState turnSimState;
    private final CANcoderSimState canCoderSimState;

    public SwerveModuleSim(LogFieldsTable fieldsTable, ModuleCanAddresses canAddresses, int moduleNum) {
        super(fieldsTable, canAddresses, moduleNum);
        driveSimState = driveMotor.getSimState();
        turnSimState = turnMotor.getSimState();
        canCoderSimState = canCoder.getSimState();

        driveSimState.Orientation = ChassisReference.CounterClockwise_Positive;
        turnSimState.Orientation = ChassisReference.Clockwise_Positive;
        canCoderSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    }

    @Override
    protected void periodicBeforeFields() {
        double battery = RobotController.getBatteryVoltage();
        driveSimState.setSupplyVoltage(battery);
        turnSimState.setSupplyVoltage(battery);
        canCoderSimState.setSupplyVoltage(battery);

        driveSim.setInputVoltage(driveSimState.getMotorVoltage());
        turnSim.setInputVoltage(turnSimState.getMotorVoltage());
        driveSim.update(ROBOT_PERIOD.in(Seconds));
        turnSim.update(ROBOT_PERIOD.in(Seconds));

        driveSimState.setRawRotorPosition(Radians.of(driveSim.getAngularPosition()).times(DRIVE_GEAR_RATIO));
        driveSimState.setRotorVelocity(RadiansPerSecond.of(driveSim.getAngularVelocity()).times(DRIVE_GEAR_RATIO));
        turnSimState.setRawRotorPosition(Radians.of(turnSim.getAngularPosition()).times(TURN_GEAR_RATIO));
        turnSimState.setRotorVelocity(RadiansPerSecond.of(turnSim.getAngularVelocity()).times(TURN_GEAR_RATIO));
        canCoderSimState.setRawPosition(Radians.of(turnSim.getAngularPosition()));
        canCoderSimState.setVelocity(RadiansPerSecond.of(turnSim.getAngularVelocity()));

        super.periodicBeforeFields();
    }
}