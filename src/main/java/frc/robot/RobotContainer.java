package frc.robot;

import java.util.Optional;

import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;

public class RobotContainer {
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
    }

    public Optional<Alliance> getAlliance() {
        return MatchState.getAlliance();
    }

    public Command getAutonomousCommand() {
        return Command.noRequirements(corotuine -> System.out.println("No Autonomous Command Configured")).named("Autonomous Command");
    }
}
