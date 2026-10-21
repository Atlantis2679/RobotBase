package frc.robot.subsystems.swerve.io;

import frc.robot.RobotMap.CANBUS.ModuleCanAddresses;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class SwerveModuleSim extends SwerveModuleIOTalonFX {
    public SwerveModuleSim(LogFieldsTable fieldsTable, ModuleCanAddresses canAddresses, int moduleNum) {
        super(fieldsTable, canAddresses, moduleNum);
    }
}
