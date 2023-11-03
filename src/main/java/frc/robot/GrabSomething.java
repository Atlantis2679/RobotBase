package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.SendableType;

public class GrabSomething extends CommandBase {
    private final SendableChooser<Integer> chooser = new SendableChooser<>();
    {
        chooser.setDefaultOption("normal", 1);
        chooser.addOption("multiplied", 2);
        chooser.addOption("tripled", 3);
    }
    private final PIDController pidController = new PIDController(0, 0, 0);
    private double number = 5;

    @Override
    public void initialize() {
        System.out.println("started");
    }

    @Override
    public void execute() {
        System.out.println("number: " + number * chooser.getSelected());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("finish");
    }

    @Override
    public boolean isFinished() {
        return (int) (Math.random() * 1000000) % 30 == 0;
    }

    public Tuneable fullTunable() {
        return (builder) -> {
            builder.setSendableType(SendableType.LIST);
            builder.addChild("run", (Sendable) super::initSendable);
            builder.addChild("value", (Tuneable) (valuesBuilder) -> {
                valuesBuilder.setSendableType(SendableType.LIST);
                valuesBuilder.addBooleanProperty("isEven", () -> number % 2 == 0, null);
                valuesBuilder.addDoubleProperty("num", () -> number, (val) -> number = val);
                valuesBuilder.addChild("choose pls", chooser);
            });
            builder.addChild("run multiplied", new InstantCommand(() -> number *= 2).andThen(this.asProxy()));
            builder.addChild("papa", pidController);
        };
    }
}
