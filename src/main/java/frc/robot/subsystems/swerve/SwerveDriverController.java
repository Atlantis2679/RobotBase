package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.DriverController.ACCELERATION_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.DriverController.ANGULAR_ACCELERATION_LIMIT;
import static frc.robot.subsystems.swerve.SwerveConstants.DriverController.MAX_ANGULAR_VELOCITY;
import static frc.robot.subsystems.swerve.SwerveConstants.DriverController.MAX_LINEAR_VELOCITY;
import static frc.robot.subsystems.swerve.SwerveConstants.DriverController.SENSITIVE_ROTATION_MULTIPLIER;
import static frc.robot.subsystems.swerve.SwerveConstants.DriverController.SENSITIVE_TRANSLATION_MULTIPLIER;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.TunableTable;

import frc.robot.RobotContainer;
import frc.robot.utils.SlewRateLimiter2d;
import frc.robot.utils.TunableCommand;

public class SwerveDriverController implements Command, ComplexTunable {
    private final Swerve swerve;

    private DoubleSupplier sidewaysSupplier;
    private DoubleSupplier forwardSupplier;
    private DoubleSupplier rotationsSupplier;
    private BooleanSupplier isFieldRelative;
    private BooleanSupplier isSensitiveMode;

    private final Selectable<Double> linearVelocityMultiplierSelect = new Selectable<>();
    private final Selectable<Double> rotationVelocityMultiplierSelect = new Selectable<>();

    private final SlewRateLimiter2d translationSlewRateLimiter = new SlewRateLimiter2d(ACCELERATION_LIMIT.in(MetersPerSecondPerSecond));
    private final SlewRateLimiter rotationSlewRateLimiter = new SlewRateLimiter(ANGULAR_ACCELERATION_LIMIT.in(DegreesPerSecondPerSecond));

    public SwerveDriverController(Swerve swerve, DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
        DoubleSupplier rotationsSupplier, BooleanSupplier isFieldRelative, BooleanSupplier isSensitiveMode) {
        this.swerve = swerve;

        this.sidewaysSupplier = sidewaysSupplier;
        this.forwardSupplier = forwardSupplier;
        this.rotationsSupplier = rotationsSupplier;
        this.isFieldRelative = isFieldRelative;
        this.isSensitiveMode = isSensitiveMode;

        linearVelocityMultiplierSelect.addDefault("REGULAR (100%)", 1.0);
        linearVelocityMultiplierSelect.add("TEENAGER (75%)", 0.75);
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
        boolean wasFieldRelative = isFieldRelative.getAsBoolean();
        resetLimiters(wasFieldRelative);

        while (true) {
            boolean fieldRelative = isFieldRelative.getAsBoolean();
            if (fieldRelative != wasFieldRelative) {
                resetLimiters(fieldRelative);
                wasFieldRelative = fieldRelative;
            }

            double precentageForward = forwardSupplier.getAsDouble() * linearVelocityMultiplierSelect.getSelected();
            double precentageSideways = sidewaysSupplier.getAsDouble() * linearVelocityMultiplierSelect.getSelected();
            double precentageRotation = rotationsSupplier.getAsDouble() * rotationVelocityMultiplierSelect.getSelected();

            if (isSensitiveMode.getAsBoolean()) {
                precentageForward *= SENSITIVE_TRANSLATION_MULTIPLIER;
                precentageSideways *= SENSITIVE_TRANSLATION_MULTIPLIER;
                precentageRotation *= SENSITIVE_ROTATION_MULTIPLIER;
            }

            Translation2d translation = translationSlewRateLimiter.calculate(
                new Translation2d(precentageForward * MAX_LINEAR_VELOCITY.in(MetersPerSecond), precentageSideways * MAX_LINEAR_VELOCITY.in(MetersPerSecond)));
            double rotation = rotationSlewRateLimiter.calculate(precentageRotation * MAX_ANGULAR_VELOCITY.in(DegreesPerSecond));

            swerve.drive(
                MetersPerSecond.of(translation.getX()),
                MetersPerSecond.of(translation.getY()),
                DegreesPerSecond.of(rotation),
                fieldRelative,
                false);
            
            coroutine.yield();
        }
    }

    private void resetLimiters(boolean fieldRelative) {
        ChassisVelocities v = fieldRelative
            ? swerve.getFieldRelativeChassisVelocities()
            : swerve.getRobotRelativeChassisVelocities();
        double flip = fieldRelative && RobotContainer.isRedAlliance() ? -1 : 1; // undo drive()'s red flip
        translationSlewRateLimiter.reset(new Translation2d(v.vx * flip, v.vy * flip));
        rotationSlewRateLimiter.reset(Math.toDegrees(v.omega));
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
        table.publish("Drive Velocity Multiplier", linearVelocityMultiplierSelect);
        table.publish("Turn Velocity Multiplier", rotationVelocityMultiplierSelect);
        table.publish("Run", new TunableCommand(this));
    }
}
