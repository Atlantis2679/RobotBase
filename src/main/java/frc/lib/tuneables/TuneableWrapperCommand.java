package frc.lib.tuneables;

import java.util.Set;
import java.util.function.Function;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TuneableWrapperCommand extends TuneableCommand {
    private final Command command;
    private final Tuneable tuneable;

    public TuneableWrapperCommand(Function<TuneablesTable, Command> commandFactory, SendableType sendableType) {
        TuneablesTable tuneablesTable = new TuneablesTable(sendableType);
        this.command = commandFactory.apply(tuneablesTable);
        this.tuneable = tuneablesTable;

        setName(command.getName());
    }

    public TuneableWrapperCommand(Function<TuneablesTable, Command> commandFactory) {
        this(commandFactory, SendableType.LIST);
    }

    public TuneableWrapperCommand(Command command, Tuneable tuneable) {
        this.command = command;
        this.tuneable = tuneable;

        setName(command.getName());
    }

    public void initialize() {
        command.initialize();
    }

    public void execute() {
        command.execute();
    }

    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    public boolean isFinished() {
        return command.isFinished();
    }

    public Set<Subsystem> getRequirements() {
        return command.getRequirements();
    }

    public boolean runsWhenDisabled() {
        return command.runsWhenDisabled();
    }

    public InterruptionBehavior getInterruptionBehavior() {
        return command.getInterruptionBehavior();
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        tuneable.initTuneable(builder);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        command.initSendable(builder);
    }
}