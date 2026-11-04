package frc.robot.subsystems.vision;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

import frc.robot.Robot;
import frc.robot.subsystems.poseestimation.TrustLevel;

public class VisionConstants {
  public static final TrustLevel TRUST_LEVEL_MULTIPLIER = new TrustLevel(0.01, 1.9);
  public static final TrustLevel PRE_MATCH_TRUST_LEVEL_MULTIPLIER = new TrustLevel(0.2, 0.05);

  public static final Distance AVG_DISTANCE_THRESHOLD = Meters.of(3);
  public static final double AMBIGUITY_THRESHOLD = 0.2;

  public record CameraConfig(String name, double stdFactor, Transform3d robotToCam, SimCameraProperties simCameraProperties) {
  };

  public static CameraConfig[] CAMERAS = {
    new CameraConfig(
      "Front", 1.0,
      new Transform3d(new Translation3d(Meters.of(0.297), Meters.of(0.155), Meters.of(0.177)),
      new Rotation3d(Degrees.of(0), Degrees.of(-48), Degrees.of(-22.5))),
      new SimCameraProperties()
    ),
    new CameraConfig(
      "Side", 1.0,
      new Transform3d(new Translation3d(Meters.of(-0.155), Meters.of(-0.2), Meters.of(0.605)),
      new Rotation3d(Degrees.of(.0), Degrees.of(5), Degrees.of(-90))),
      new SimCameraProperties()
    )
  };

  public static class Sim {
    public static VisionSystemSim VISION_SIM = Robot.isSimulation() ? new VisionSystemSim("VisionSim") : null;
  }
}
