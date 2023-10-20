package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.RefValue;
import frc.lib.tuneables.TuneablesTable;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        TuneablesTable tuneables = new TuneablesTable();
        RefValue<Boolean> isOn = new RefValue<Boolean>(false);
        tuneables.addBoolean("hello", () -> isOn.value, (value) -> {
            System.out.println(value);
            isOn.value = value;
        });
        System.out.println("should publish now");
        tuneables.publish("workinggg", SmartDashboard::putData);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
