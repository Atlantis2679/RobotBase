package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RefValue;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.TuneablesTable;
import frc.lib.tuneables.wrappers.PIDControllerTuneable;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        TuneablesTable tuneables = new TuneablesTable(SendableType.LIST);

        RefValue<Boolean> printButton = tuneables.addBoolean("print", false);
        RefValue<Double> num = tuneables.addNumber("num", 0.3);
        RefValue<String> str = tuneables.addString("str", "write here!");

        TuneablesTable innerTable = new TuneablesTable(SendableType.LIST);
        RefValue<Boolean> innerBool = innerTable.addBoolean("innerBool", true);
        RefValue<Double> innerNum = innerTable.addNumber("innerNum", 3.0);
        RefValue<String> innerStr = innerTable.addString("innerStr", "something");
        tuneables.addTuneable("inner", innerTable);

        innerTable.addTuneable("pid", new PIDControllerTuneable(0.4, 0.3, 0.2));

        tuneables.publish("workinggg");

        new Trigger(printButton::get).onTrue(new InstantCommand(() -> {
            System.out.println("num: " + num.get() + ", str: " + str.get() + ", innerBool: " + innerBool.get() + ", innerNum:" + innerNum.get()
                    + ", innerStr: " + innerStr.get());
            printButton.set(false);
        }).ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
