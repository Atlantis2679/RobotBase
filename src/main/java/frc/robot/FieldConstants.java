package frc.robot;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose3d;

public class FieldConstants {
    public static final Field FIELD = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

    public static final double HEIGHT_ABOVE_FIELD_THRESHOLD_METERS = 0.05;
    public static final double PITCH_ROLL_THRESHOLD_DEGRESS = 10;

    public static boolean isOnField(Pose3d pose) {
        if (Math.abs(pose.getZ()) > HEIGHT_ABOVE_FIELD_THRESHOLD_METERS) return false;
        if (Math.toDegrees(Math.abs(pose.getRotation().getX())) > PITCH_ROLL_THRESHOLD_DEGRESS) return false;
        if (Math.toDegrees(Math.abs(pose.getRotation().getY())) > PITCH_ROLL_THRESHOLD_DEGRESS) return false;  
        if (pose.getX() < 0 || pose.getY() < 0) return false;
        if (pose.getX() > FIELD.length || pose.getY() > FIELD.width) return false;
        return true;
    }
}
