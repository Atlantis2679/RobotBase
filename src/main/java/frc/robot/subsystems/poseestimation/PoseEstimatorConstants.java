package frc.robot.subsystems.poseestimation;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;

public final class PoseEstimatorConstants {
    public static final TrustLevel ODOMETRY_STD_DEVS = new TrustLevel(0.003,0.002);
    public static final Time ODOMETRY_POSES_BUFFER_SIZE = Units.Seconds.of(2);
}
