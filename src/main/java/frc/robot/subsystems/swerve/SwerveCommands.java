package frc.robot.subsystems.swerve;


import static frc.robot.subsystems.swerve.SwerveConstants.Modules.MAX_DRIVE_VELOCITY;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.getModuleName;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Second;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.sysid.SysIdRoutineLog.State;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Current;

import frc.robot.subsystems.swerve.SwerveConstants.SysId;

public class SwerveCommands {
    private final SysIdRoutineLog sysIdLog = new SysIdRoutineLog("swerve-drive");
    private final Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
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

    public Command stop() {
        return swerve.runRepeatedly(swerve::stop).named("Stop");
    }

    public Command sysIdQuasistatic(boolean forward) {
        return sysIdTest(forward ? State.QUASISTATIC_FORWARD : State.QUASISTATIC_REVERSE,
            t -> Amps.of(SysId.RAMP_RATE.in(Amps.per(Second)) * t * (forward ? 1 : -1)));
    }

    public Command sysIdDynamic(boolean forward) {
        double sign = forward ? 1 : -1;
        return sysIdTest(forward ? State.DYNAMIC_FORWARD : State.DYNAMIC_REVERSE,
            t -> SysId.STEP_CURRENT.times(sign));
    }

    private Command sysIdTest(State state, DoubleFunction<Current> currentAtTime) {
        return swerve.run(coroutine -> {
            Timer timer = Timer.createStarted();
            while (!timer.hasElapsed(SysId.TIMEOUT)) {
                swerve.runDriveCharacterization(currentAtTime.apply(timer.get()));
                logSysId(state);
                coroutine.yield();
            }
            stopSysId();
        }).whenCanceled(this::stopSysId).named("SysId " + state);
    }

    private void logSysId(State state) {
        for (SwerveModule m : swerve.getModules()) {
            sysIdLog.motor("drive-" + getModuleName(m.getModuleNum()))
                .value("voltage", m.getCurrent().in(Amps), Amps.name())
                .linearPosition(m.getDriveDistance())
                .linearVelocity(m.getDriveVelocity());
        }
        sysIdLog.recordState(state);
    }

    private void stopSysId() {
        swerve.stop();;
        sysIdLog.recordState(State.NONE);
    }
}
