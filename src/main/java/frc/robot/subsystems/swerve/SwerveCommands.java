package frc.robot.subsystems.swerve;


import static frc.robot.subsystems.swerve.SwerveConstants.Modules.MAX_VELOCITY;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

public class SwerveCommands {
    private final Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    public SwerveDriverController driverController(DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
        DoubleSupplier rotationSupplier, BooleanSupplier isFieldRelativeSupplier, BooleanSupplier isSensetiveMode) {

        return new SwerveDriverController(swerve, forwardSupplier, sidewaysSupplier, rotationSupplier,
            isFieldRelativeSupplier, isSensetiveMode);
    }

    public Command driveForward(DoubleSupplier forwardPrecentageSupplier) {
        return swerve.run((_) -> swerve.drive(MAX_VELOCITY.times(forwardPrecentageSupplier.getAsDouble()), MetersPerSecond.zero(), DegreesPerSecond.zero(), false, true))
            .named("Drive Forward");
    }

    public Command xWheelLock() {
        return swerve.run((_) -> {
            SwerveModuleVelocity[] moduleVelocities = new SwerveModuleVelocity[4];
            moduleVelocities[0] = new SwerveModuleVelocity(0, Rotation2d.fromDegrees(45));
            moduleVelocities[1] = new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-45));
            moduleVelocities[2] = new SwerveModuleVelocity(0, Rotation2d.fromDegrees(135));
            moduleVelocities[3] = new SwerveModuleVelocity(0, Rotation2d.fromDegrees(-135));

            swerve.setModulesVelocities(moduleVelocities, true, false, false);
        }).named("X Wheel Lock");
    }

    public SwerveControlModulesCommand controlModules(DoubleSupplier turnXSupplier, DoubleSupplier turnYSupplier,
            DoubleSupplier velocitySupplier) {
        return new SwerveControlModulesCommand(swerve, turnXSupplier, turnYSupplier, velocitySupplier);
    }

    public Command stop() {
        return swerve.run((coroutine) -> {
            while (true) {
                swerve.stop();
                coroutine.yield();
            }
        }).named("Stop");
    }
}
