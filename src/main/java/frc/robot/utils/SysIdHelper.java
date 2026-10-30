package frc.robot.utils;

import java.util.function.BooleanSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.command3.sysid.SysIdRoutine.Direction;
import org.wpilib.driverstation.RobotState;

import com.ctre.phoenix6.SignalLogger;

public class SysIdHelper {
    public static Command sysIdSequence(String name, SysIdRoutine r, BooleanSupplier nextStage) {
        return Command.noRequirements(co -> {
            SignalLogger.start();
            for (Command stage : new Command[] {
                    r.quasistatic(Direction.FORWARD), r.quasistatic(Direction.REVERSE),
                    r.dynamic(Direction.FORWARD), r.dynamic(Direction.REVERSE)}) {
                co.waitUntil(nextStage);
                co.await(stage.until(nextStage).named(stage.name()));
            }
        })
        .until(RobotState::isDisabled)
        .whenExited(SignalLogger::stop)
        .named("SysId " + name);
    }
}
