package frc.robot.subsystems.swerve;


import static frc.robot.subsystems.swerve.SwerveConstants.Modules.MAX_DRIVE_VELOCITY;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.sysid.SysIdRoutine;
import org.wpilib.command3.sysid.SysIdRoutine.SysIdMechanism;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import com.ctre.phoenix6.SignalLogger;

import frc.robot.subsystems.swerve.SwerveConstants.SysId;
import frc.robot.utils.SysIdHelper;

public class SwerveCommands {
    private final Swerve swerve;
    private final SysIdRoutine sysIdRoutine;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.per(Second).of(SysId.RAMP_RATE.in(Amps.per(Second))), Volts.of(SysId.STEP_CURRENT.in(Amps)), SysId.TIMEOUT, (state) -> SignalLogger.writeString("swerve-sysid-state", state.toString())),
            new SysIdMechanism((current) -> swerve.runDriveCharacterization(Amps.of(current.in(Volts))), null, swerve));
    }

    public SwerveDriverController driverController(DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
        DoubleSupplier rotationSupplier, BooleanSupplier isFieldRelativeSupplier, BooleanSupplier isSensitiveMode) {

        return new SwerveDriverController(swerve, forwardSupplier, sidewaysSupplier, rotationSupplier,
            isFieldRelativeSupplier, isSensitiveMode);
    }

    public Command driveForward(DoubleSupplier forwardPrecentageSupplier) {
        return swerve.runRepeatedly(() -> swerve.drive(MAX_DRIVE_VELOCITY.times(forwardPrecentageSupplier.getAsDouble()), MetersPerSecond.zero(), DegreesPerSecond.zero(), false, true))
            .named("Drive Forward");
    }

    public Command xWheelLock() {
        return swerve.runRepeatedly(() -> {
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

    public Command sysId(BooleanSupplier nextStage) {
        return SysIdHelper.sysIdSequence("Swerve SysId", sysIdRoutine, nextStage);
    }

    public Command stop() {
        return swerve.runRepeatedly(swerve::stop).named("Stop");
    }
}
