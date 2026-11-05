package frc.robot.subsystems.vision;

import static org.wpilib.units.Units.Meters;
import static frc.robot.FieldConstants.FIELD;
import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.AVG_DISTANCE_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.vision.VisionConstants.PRE_MATCH_TRUST_LEVEL_MULTIPLIER;
import static frc.robot.subsystems.vision.VisionConstants.TRUST_LEVEL_MULTIPLIER;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.fields.FieldTag;
import org.wpilib.framework.RobotBase;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.measure.Distance;

import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.poseestimation.TrustLevel;
import frc.robot.subsystems.poseestimation.TunableTrustLevel;
import frc.robot.subsystems.poseestimation.PoseEstimator.VisionMeasurement;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionConstants.Sim;
import frc.robot.subsystems.vision.io.VisionAprilTagsIO;
import frc.robot.subsystems.vision.io.VisionAprilTagsIO.VisionData;
import frc.robot.utils.TunableMeasure;
import frc.robot.utils.MathUtils.Bounds;
import frc.robot.subsystems.vision.io.VisionAprilTagsIOPhoton;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

import static frc.robot.utils.MathUtils.inRange;

public class Vision implements ComplexTunable {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("Vision");
  private final VisionAprilTagsIO[] visionCameras = new VisionAprilTagsIO[CAMERAS.length];

  private static final TunableTrustLevel trustLevelMultiplier = new TunableTrustLevel(TRUST_LEVEL_MULTIPLIER);
  private static final TunableTrustLevel preMatchTrustLevelMultiplier = new TunableTrustLevel(PRE_MATCH_TRUST_LEVEL_MULTIPLIER);
  private static final TunableDouble ambiguityThreshold = TunableDouble.create(AMBIGUITY_THRESHOLD);
  private static final TunableMeasure<Distance> distanceThreshold = new TunableMeasure<Distance>(AVG_DISTANCE_THRESHOLD, Meters);

  public Vision() {
    if (Robot.isSimulation()) {
      Telemetry.log("VisionSimulation", Sim.VISION_SIM.getDebugField());
    for (FieldTag tag : FIELD.getTags())
        Sim.VISION_SIM.addVisionTargets("apriltag", new VisionTargetSim(
            FIELD.getTagPose(tag.getID()).get(), TargetModel.kAprilTag36h11, tag.getID()));
    }
    PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("Vision Alerts");
    for (int i = 0; i < visionCameras.length; i++) {
      CameraConfig camera = CAMERAS[i];
      visionCameras[i] = new VisionAprilTagsIOPhoton(fieldsTable, camera, alertsGroup);
      VisionAprilTagsIO io = visionCameras[i];
      alertsGroup.addMediumAlert(() -> camera.name() + " Disconnected!", () -> !io.isConnected.getAsBoolean());
    }
    Scheduler.getDefault().addPeriodic(this::periodic);
  }

  private static List<VisionMeasurement> getAllResultsInIO(VisionAprilTagsIO io) {
    VisionData[] visionDataArr = io.visionData.get();
    List<VisionMeasurement> visionMeasurements = new ArrayList<>();
    double stdFactor = io.getCameraConfig().stdFactor();
    for (VisionData visionData : visionDataArr) {
      if (!inRange(visionData.ambiguity(), 0, ambiguityThreshold.get())) continue;
      if (!Bounds.of(Meters.zero(), distanceThreshold.get()).contains(visionData.avgDistanceToCam())) continue;
      if (!FieldConstants.isOnField(visionData.robotPose())) continue;
      TrustLevel trustLevels = calculateTrustLevel(stdFactor, visionData.tagsUsed(), visionData.avgDistanceToCam().in(Meters), visionData.ambiguity());
      visionMeasurements.add(new VisionMeasurement(visionData.robotPose().toPose2d(), trustLevels, visionData.timestamp()));
    }
    return visionMeasurements;
  }

  private List<VisionMeasurement> getAllResults() {
    List<VisionMeasurement> measurements = new ArrayList<>();
    for (VisionAprilTagsIO io : visionCameras) {
      for (VisionMeasurement measurement : getAllResultsInIO(io)) {
        measurements.add(measurement);
      }
    }
    fieldsTable.recordOutput("Vision measurements", measurements.toArray(new VisionMeasurement[0]));
    return measurements;
  }

  public void periodic() {
    if (RobotBase.isSimulation()) {
      VisionConstants.Sim.VISION_SIM.update(PoseEstimator.getInstance().getOdometryPose());
    }
    for (VisionMeasurement measurement : getAllResults()) {
      PoseEstimator.getInstance().addVisionMeasurement(measurement);
    }
  }

  private static TrustLevel calculateTrustLevel(double stdFactor, int tagsUsed, double avgDistanceToCamMeters, double ambiguity) {
    if (ambiguity == 1 || tagsUsed == 0 || avgDistanceToCamMeters == 0)
      return new TrustLevel(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    double value = Math.pow(avgDistanceToCamMeters, 2) / Math.pow(tagsUsed, 2) / Math.pow(1 - ambiguity, 2) * stdFactor;
    TrustLevel result = new TrustLevel(value, value);
    boolean robotDisabled = RobotState.isDisabled();
    if(robotDisabled) result.multiply(PRE_MATCH_TRUST_LEVEL_MULTIPLIER);

    double thetaStd;
    if (RobotState.isDisabled()) thetaStd = tagsUsed > 1 ? result.rotationStdDev() : Double.POSITIVE_INFINITY;
    else if (PoseEstimator.getInstance().hasGyroAngle()) thetaStd = Double.POSITIVE_INFINITY;
    else thetaStd = result.rotationStdDev();
    return new TrustLevel(result.xyStdDev(), thetaStd);  }

  @Override
  public void publishTunable(TunableTable table) {
    table.publish("Trust level multiplier", trustLevelMultiplier);
    table.publish("Distance threshold", distanceThreshold.tunable());
    table.publish("PreMatch trust level multiplier", preMatchTrustLevelMultiplier);
    table.publishDouble("Ambiguity threshold", ambiguityThreshold::get, ambiguityThreshold::set);
  }
}
