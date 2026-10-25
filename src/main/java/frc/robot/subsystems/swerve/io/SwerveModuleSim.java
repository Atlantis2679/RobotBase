package frc.robot.subsystems.swerve.io;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.DRIVE_GEAR_RATIO;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.DRIVE_MOTOR;

import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;

import frc.robot.RobotMap.CANBUS.ModuleCanAddresses;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class SwerveModuleSim extends SwerveModuleIOTalonFX {
    LinearSystem<N2, N1, N2> plant = Models.singleJointedArmFromPhysicalConstants(DRIVE_MOTOR, 0.0, DRIVE_GEAR_RATIO);
    private DCMotorSim driveMotorSim = new DCMotorSim(plant, DRIVE_MOTOR);

    public SwerveModuleSim(LogFieldsTable fieldsTable, ModuleCanAddresses canAddresses, int moduleNum) {
        super(fieldsTable, canAddresses, moduleNum);
    }
}
