package frc.robot.utils;

import static org.wpilib.util.ErrorMessages.requireNonNullParam;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Scheduler.ScheduleResult;
import org.wpilib.telemetry.TelemetryLoggable;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableOption;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.tunable.TunableTable;

public class TunableCommand implements TelemetryLoggable, ComplexTunable {
    private final Command command;
    private final Scheduler scheduler;

    public TunableCommand(Command command) {
        this(Scheduler.getDefault(), command);
    }

    public TunableCommand(Scheduler scheduler, Command command) {
        this.scheduler = requireNonNullParam(scheduler, "scheduler", "TunableCommand");
        this.command = requireNonNullParam(command, "command", "TunableCommand");
    }

    public Command getCommand() {
        return command;
    }

    public Scheduler getScheduler() {
        return scheduler;
    }

    @Override
    public void logTo(TelemetryTable table) {
        table.log("name", command.name());
        table.log("requirements", command.requirements().stream().map(Mechanism::getName).toArray(String[]::new));
        table.log("running", scheduler.isScheduledOrRunning(command));
    }

    @Override
    public String getTelemetryType() {
        return "Command";
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publishValue(
            "name",
            command::name,
            (_) -> {},
            String.class,
            TunableConfig.of(TunableOption.IMMUTABLE, TunableOption.GET_ON_CHANGE));
        table.publishBoolean(
            "running",
            () -> scheduler.isScheduledOrRunning(command),
            value -> {
                if (value) {
                    ScheduleResult result = scheduler.schedule(command);
                    if (!result.successful()) {
                        TunableRegistry.reportWarning(
                            "Failed to schedule " + command.name() + ": " + result);
                    }
                } else {
                    scheduler.cancel(command);
                }
            },
            TunableConfig.of(TunableOption.ALWAYS_GET));
    }

    @Override
    public String getTunableType() {
        return "Command";
    }
}
