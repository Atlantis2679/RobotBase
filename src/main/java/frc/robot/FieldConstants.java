package frc.robot;

import static org.wpilib.units.Units.Centimeters;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

public class FieldConstants {
    public static final Field FIELD = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

    public static final Distance HEIGHT_ABOVE_FIELD_THRESHOLD = Centimeters.of(5);
    public static final Angle PITCH_ROLL_THRESHOLD = Degrees.of(10);

    public static boolean isOnField(Pose3d pose) {
        if (pose.getTranslation().getMeasureZ().isNear(Meters.zero(), HEIGHT_ABOVE_FIELD_THRESHOLD)) return false;
        if (pose.getRotation().getMeasureX().isNear(Degrees.zero(), PITCH_ROLL_THRESHOLD)) return false;
        if (pose.getRotation().getMeasureY().isNear(Degrees.zero(), PITCH_ROLL_THRESHOLD)) return false;
        if (pose.getX() < 0 || pose.getY() < 0) return false;
        if (pose.getX() > FIELD.length || pose.getY() > FIELD.width) return false;
        return true;
    }
}
