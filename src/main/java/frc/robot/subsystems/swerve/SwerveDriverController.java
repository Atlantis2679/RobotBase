package frc.robot.subsystems.swerve;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableTable;

import frc.robot.utils.TunableCommand;

import static frc.robot.subsystems.swerve.SwerveConstants.DriverController.*;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;

public class SwerveDriverController implements Command, ComplexTunable {
    private final Swerve swerve;

    private DoubleSupplier sidewaysSupplier;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier rotationsSupplier;
    private BooleanSupplier isFieldRelative;
    private BooleanSupplier isSensetiveMode;

    private final Selectable<Double> linearVelocityMultiplierSelect = new Selectable<>();
    private final Selectable<Double> rotationVelocityMultiplierSelect = new Selectable<>();

    private final SlewRateLimiter forwardSlewRateLimiter = new SlewRateLimiter(ACCELERATION_LIMIT.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter sidewaysSlewRateLimiter = new SlewRateLimiter(ACCELERATION_LIMIT.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(ANGULAR_ACCELERATION_LIMIT.in(DegreesPerSecondPerSecond));

    public SwerveDriverController(Swerve swerve, DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
        DoubleSupplier rotationsSupplier, BooleanSupplier isFieldRelative, BooleanSupplier isSensetiveMode) {
        this.swerve = swerve;

        this.sidewaysSupplier = sidewaysSupplier;
        this.forwardSupplier = forwardSupplier;
        this.rotationsSupplier = rotationsSupplier;
        this.isFieldRelative = isFieldRelative;
        this.isSensetiveMode = isSensetiveMode;

        linearVelocityMultiplierSelect.addDefault("REGULAR (100%)", 1.0);
        linearVelocityMultiplierSelect.add("TEENEGER (75%)", 0.75);
        linearVelocityMultiplierSelect.add("CHILD (50%)", 0.5);
        linearVelocityMultiplierSelect.add("BABY (30%)", 0.3);
        linearVelocityMultiplierSelect.add("EGG (10%)", 0.1);

        rotationVelocityMultiplierSelect.addDefault("REGULAR (100%)", 1.0);
        rotationVelocityMultiplierSelect.add("TEENEGER (75%)", 0.75);
        rotationVelocityMultiplierSelect.add("CHILD (50%)", 0.5);
        rotationVelocityMultiplierSelect.add("BABY (30%)", 0.3);
        rotationVelocityMultiplierSelect.add("EGG (10%)", 0.1);
    }

    @Override
    public void run(Coroutine coroutine) {
        double velocityMultiplier = linearVelocityMultiplierSelect.getSelected();

        double precentageForward = forwardSupplier.getAsDouble() * velocityMultiplier;
        double precentageSideways = sidewaysSupplier.getAsDouble() * velocityMultiplier;
        double precentageRotation = rotationsSupplier.getAsDouble() * velocityMultiplier;

        if (isSensetiveMode.getAsBoolean()) {
            precentageForward *= SENSETIVE_TRANSLATION_MULTIPLIER;
            precentageSideways *= SENSETIVE_TRANSLATION_MULTIPLIER;
            precentageRotation *= SENSETIVE_ROTATION_MULTIPLIER;
        }

        swerve.drive(
            MetersPerSecond.of(forwardSlewRateLimiter.calculate(precentageForward * MAX_LINEAR_VELOCITY.in(MetersPerSecond))),
            MetersPerSecond.of(sidewaysSlewRateLimiter.calculate(precentageSideways * MAX_LINEAR_VELOCITY.in(MetersPerSecond))),
            DegreesPerSecond.of(rotationSlewRateLimiter.calculate(precentageRotation * MAX_ANGULAR_VELOCITY.in(DegreesPerSecond))),
            isFieldRelative.getAsBoolean(),
            false);
    }

    @Override
    public String name() {
        return "Driver Controller";
    }

    @Override
    public Set<Mechanism> requirements() {
        return Set.of(swerve);
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publish("Velocity Multiplier Selector", linearVelocityMultiplierSelect);
        table.publish("Run", new TunableCommand(this));
    }
}
