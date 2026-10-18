package frc.robot.subsystems.swerve.io;

import static org.wpilib.units.Units.Minutes;
import static org.wpilib.units.Units.Seconds;
import static frc.robot.Constants.ROBOT_PERIOD;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.*;
import static frc.robot.subsystems.swerve.SwerveConstants.Sim.*;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.FlywheelSim;

import frc.robot.utils.MathUtils.Bounds;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class SwerveModuleSim extends SwerveModuleIO {
    private final FlywheelSim driveMotor;
    private final FlywheelSim turnMotor;
    private final Bounds angleBounds = new Bounds(-1, 1);
    private double angleRotaions = 0;
    private double driveMotorRotations = 0;

    private PIDController turnPIDController = new PIDController(SIM_TURN_MOTOR_KP, SIM_TURN_MOTOR_KI,
            SIM_TURN_MOTOR_KD);

    public SwerveModuleSim(LogFieldsTable fieldsTable) {
        super(fieldsTable);

        DCMotor motorsModel = DCMotor.getFalcon500(1);

        turnPIDController.enableContinuousInput(-0.5, 0.5);

        driveMotor = new FlywheelSim(
                Models.flywheelFromPhysicalConstants(motorsModel, DRIVE_MOTOR_MOMENT_OF_INERTIA, DRIVE_GEAR_RATIO),
                motorsModel);
        turnMotor = new FlywheelSim(
                Models.flywheelFromPhysicalConstants(motorsModel, TURN_MOTOR_MOMENT_OF_INERTIA, TURN_GEAR_RATIO),
                motorsModel);
    }

    @Override
    protected void periodicBeforeFields() {
        driveMotor.update(ROBOT_PERIOD.in(Seconds));
        turnMotor.update(ROBOT_PERIOD.in(Seconds));

        driveMotorRotations += Units.radiansPerSecondToRotationsPerMinute(driveMotor.getAngularVelocity()) * ROBOT_PERIOD.in(Minutes);
        angleRotaions += Units.radiansPerSecondToRotationsPerMinute(turnMotor.getAngularVelocity()) * ROBOT_PERIOD.in(Minutes);

        turnMotor.setInputVoltage(turnPIDController.calculate());
    }

    @Override
    public void setCoast() {
        driveMotor.setInputVoltage(0);
    }

}
