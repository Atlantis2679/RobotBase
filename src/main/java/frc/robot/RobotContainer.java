package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.RefValue;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.TuneablesTable;

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

        TuneablesTable pidTable = new TuneablesTable(SendableType.PID);
        RefValue<Double> p = pidTable.addNumber("p", 4.0);
        RefValue<Double> i = pidTable.addNumber("i", 2.0);
        RefValue<Double> d = pidTable.addNumber("d", 9.0);
        RefValue<Double> setpoint = pidTable.addNumber("setpoint", 100.0);
        innerTable.addTuneable("pid", pidTable);

        tuneables.publish("workinggg");

        new Trigger(printButton::get).onTrue(new InstantCommand(() -> {
            System.out.println("num: " + num.get() + ", str: " + str.get() + ", innerBool: " + innerBool.get() + ", innerNum:" + innerNum.get()
                    + ", innerStr: " + innerStr.get() + ", p: " + p.get() + ", i: " + i.get() + ", d: " + d.get() + ", setpoint: " + setpoint.get());
            printButton.set(false);
        }).ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
