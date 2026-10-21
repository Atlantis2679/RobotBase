package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import frc.robot.utils.MathUtils.Bounds;

import static frc.robot.Constants.ROBOT_PERIOD;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilogram;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerMinute;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

public final class SwerveConstants {
    public static final class Modules {
        public static final String getModuleName(int moduleNum) {
            switch (moduleNum) {
                case 0:
                    return "FL";
                case 1:
                    return "FR";
                case 2:
                    return "BL";
                case 3:
                    return "BR";
                default:
                    return "None";
            }
        }

        public static final Bounds<Angle> TURN_ANGLE_BOUNDS = Bounds.of(Degrees.zero(), Degrees.of(360));

        public static final double TURN_MOTOR_KP = 1.8 * 12;
        public static final double TURN_MOTOR_KI = 0;
        public static final double TURN_MOTOR_KD = 0;

        public static final Voltage MAX_VOLTAGE = Volts.of(12);

        public static final double PREVENT_JITTERING_MULTIPLIER = 0.01;

        public static final double DRIVE_GEAR_RATIO = 6.12;
        public static final double TURN_GEAR_RATIO = 150.0 / 7;

        public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
        public static final DCMotor TURN_MOTOR  = DCMotor.getFalcon500Foc(1);

        public static final Distance WHEEL_RADIUS = Inches.of(2);
        public static final Distance WHEEL_CIRCUMFERENCE = WHEEL_RADIUS.times(2 * Math.PI);

        public static final AngularVelocity DRIVE_MOTOR_MAX_VELOCITY = RotationsPerMinute.of(5784).div(DRIVE_GEAR_RATIO);
        public static final LinearVelocity MAX_DRIVE_VELOCITY = MetersPerSecond.of(DRIVE_MOTOR_MAX_VELOCITY.in(RadiansPerSecond) * WHEEL_CIRCUMFERENCE.in(Meters));

        public static final Angle[] OFFSETS = {
            Degrees.of(26.015625 + 90),
            Degrees.of(-179.47265625 + 90),
            Degrees.of(-41.8359375 + 90),
            Degrees.of(-117.24609375 + 270),
        };

        public static final Current DRIVE_STATOR_CURRENT_LIMIT = Amps.of(60);

        public static final Current DRIVE_SUPPLY_CURRENT_LIMIT = Amps.of(60);
        public static final Current DRIVE_SUPPLY_CURRENT_LOWER_LIMIT = Amps.of(30);
        public static final Time DRIVE_SUPPLY_CURRENT_LOWER_TIME = Seconds.of(1);

        public static final Current TURN_STATOR_CURRENT_LIMIT = Amps.of(30);
    }

    public static final Bounds<Angle> GYRO_ANGLE_BOUNDS = Bounds.of(Degrees.of(0), Degrees.of(360));

    // From Module Axis of rotation to the next, not the whole chassis size
    public static final Distance TRACK_LENGTH = Meters.of(0.557);
    public static final Distance TRACK_WIDTH = Meters.of(0.557);

    public static final Translation2d[] MODULES_LOCATIONS = {
        new Translation2d(TRACK_LENGTH.div(2), TRACK_WIDTH.div(2)),
        new Translation2d(TRACK_LENGTH.div(2), TRACK_WIDTH.div(2).unaryMinus()),
        new Translation2d(TRACK_LENGTH.div(2).unaryMinus(), TRACK_WIDTH.div(2)),
        new Translation2d(TRACK_LENGTH.div(2).unaryMinus(), TRACK_WIDTH.div(2).unaryMinus()),
    };

    public static final Time GYRO_CONNECTED_DEBUNCE_TIME = ROBOT_PERIOD.times(5);

    public static final class PathPlanner {
        public static final double FRICTION_WITH_CARPET = 1;
        public static final Mass ROBOT_MASS_KG = Kilogram.of(59.4);
        public static final MomentOfInertia MOMENT_OF_INERTIA = KilogramSquareMeters.of(4.3);

        public static final ModuleConfig MODULES_CONFIG = new ModuleConfig(
            Modules.WHEEL_RADIUS,
            Modules.MAX_DRIVE_VELOCITY,
            FRICTION_WITH_CARPET,
            Modules.DRIVE_MOTOR,
            Modules.DRIVE_SUPPLY_CURRENT_LIMIT,
            1);

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
            PathPlanner.ROBOT_MASS_KG,
            PathPlanner.MOMENT_OF_INERTIA,
            MODULES_CONFIG, MODULES_LOCATIONS);

        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0, 0, 0);
        public static final PIDConstants ROTATION_PID = new PIDConstants(0, 0, 0);

        public static final PathFollowingController FOLLOWING_CONTROLLER = new PPHolonomicDriveController(
            TRANSLATION_PID, ROTATION_PID);
    }

    public static final class DriverController {
        public static final LinearAcceleration ACCELERATION_LIMIT = MetersPerSecondPerSecond.of(4.5);
        public static final AngularAcceleration ANGULAR_ACCELERATION_LIMIT = DegreesPerSecondPerSecond.of(720);
        public static final AngularVelocity MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(Modules.MAX_DRIVE_VELOCITY.in(MetersPerSecond) / MODULES_LOCATIONS[0].getNorm());
        public static final LinearVelocity MAX_LINEAR_VELOCITY = Modules.MAX_DRIVE_VELOCITY;
        public static final double SENSETIVE_TRANSLATION_MULTIPLIER = 0.3;
        public static final double SENSETIVE_ROTATION_MULTIPLIER = 0.3;
    }
}
