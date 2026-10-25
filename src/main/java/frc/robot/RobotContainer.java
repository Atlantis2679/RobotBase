package frc.robot;

import static frc.robot.Constants.DRIVER_CONTROLLER_PORT;
import static frc.robot.Constants.OPERATOR_CONTROLLER_PORT;

import java.util.Optional;

import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.tunable.Tunables;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveDriverController;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    Swerve swerve = new Swerve();

    SwerveCommands swerveCommands = new SwerveCommands(swerve);

    NaturalXboxController driverController = new NaturalXboxController(DRIVER_CONTROLLER_PORT);
    NaturalXboxController operatorController = new NaturalXboxController(OPERATOR_CONTROLLER_PORT);

    public RobotContainer() {
        configureDriverBindings();
        configureOperatorBindings();
        publishTunables();
    }

    private void configureDriverBindings() {
        SwerveDriverController driveCommand = swerveCommands.driverController(
            driverController::getLeftY,
            driverController::getLeftX,
            driverController::getRightX,
            driverController.leftBumper().negate()::getAsBoolean,
            driverController.rightBumper()::getAsBoolean);
        
        swerve.setDefaultCommand(driveCommand);
        Tunables.publish("Swerve/Drive Command", driveCommand);
    }

    private void configureOperatorBindings() {
    }

    public static boolean isRedAlliance() {
        return getAlliance().isPresent() && getAlliance().get() == Alliance.RED;
    }

    public static Optional<Alliance> getAlliance() {
        return MatchState.getAlliance();
    }

    public Command getAutonomousCommand() {
        return Command.noRequirements(corotuine -> System.out.println("No Autonomous Command Configured")).named("Autonomous Command");
    }

    public void publishTunables() {
        Tunables.publish("Swerve", swerve);
    }
}
