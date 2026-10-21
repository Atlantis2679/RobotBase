package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.MAX_DRIVE_VELOCITY;

import java.util.Set;
import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableTable;

import frc.robot.utils.TunableCommand;

public class SwerveControlModulesCommand implements Command, ComplexTunable {
    private final Swerve swerve;
    private final DoubleSupplier turnXSupplier;
    private final DoubleSupplier turnYSupplier;
    private final DoubleSupplier velocitySupplier;
    private final TunableBoolean optimizeState = TunableBoolean.create();

    public SwerveControlModulesCommand(Swerve swerve, DoubleSupplier turnXSupplier, DoubleSupplier turnYSupplier,
            DoubleSupplier velocitySupplier) {
        this.swerve = swerve;
        this.turnXSupplier = turnXSupplier;
        this.turnYSupplier = turnYSupplier;
        this.velocitySupplier = velocitySupplier;
    }

    @Override
    public void run(Coroutine coroutine) {
        while (true) {
            double turnY = turnYSupplier.getAsDouble();
            double turnX = turnXSupplier.getAsDouble();
            double velocity = velocitySupplier.getAsDouble();

            SwerveModuleVelocity[] moduleVelocities = new SwerveModuleVelocity[4];
            double turnAngleRadians = turnX != 0 || turnY != 0
                ? Math.atan2(turnY, turnX) - Math.toRadians(90)
                : 0;
            for (int i = 0; i < moduleVelocities.length; i++) {

                moduleVelocities[i] = new SwerveModuleVelocity(
                    MAX_DRIVE_VELOCITY.times(velocity),
                    new Rotation2d(turnAngleRadians));
            }
            swerve.setModulesVelocities(moduleVelocities, optimizeState.get(), false, false);

            coroutine.yield();
        }
    }

    @Override
    public String name() {
        return "Control Modules";
    }

    @Override
    public Set<Mechanism> requirements() {
        return Set.of(swerve);
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publish("Optimize State", optimizeState);
        table.publish("Run", new TunableCommand(this));
    }
}
