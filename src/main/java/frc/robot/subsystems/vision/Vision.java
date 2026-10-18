package frc.robot.subsystems.vision;

import static org.wpilib.units.Units.Seconds;
import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.AVG_DISTANCE_THRESHOLD_METERS;
import static frc.robot.subsystems.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.vision.VisionConstants.PRE_MATCH_TRUST_LEVEL_MULTIPLIER;
import static frc.robot.subsystems.vision.VisionConstants.TRUST_LEVEL_MULTIPLIER;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.framework.RobotBase;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableTable;

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
  private static final TunableDouble distanceThresholdMeters = TunableDouble.create(AVG_DISTANCE_THRESHOLD_METERS);

  public Vision() {
    if (Robot.isSimulation()) {
      Telemetry.log("VisionSimulation", Sim.VISION_SIM.getDebugField());
    }
    PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("Vision");
    for (int i = 0; i < visionCameras.length; i++) {
      CameraConfig camera = CAMERAS[i];
      visionCameras[i] = new VisionAprilTagsIOPhoton(fieldsTable, camera);
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
      if (!inRange(visionData.avgDistanceToCam(), 0, distanceThresholdMeters.get())) continue;
      if (!FieldConstants.isOnField(visionData.robotPose())) continue;
      TrustLevel trustLevels = calculateTrustLevel(stdFactor, visionData.tagsUsed(), visionData.avgDistanceToCam(), visionData.ambiguity());
      visionMeasurements.add(new VisionMeasurement(visionData.robotPose().toPose2d(), trustLevels, Seconds.of(visionData.timestampSeconds())));
    }
    return visionMeasurements;
  }

  private List<VisionMeasurement> getAllResults() {
    List<VisionMeasurement> measurements = new ArrayList<>();
    for (VisionAprilTagsIO io : visionCameras) {
      for (VisionMeasurement measurment : getAllResultsInIO(io)) {
        measurements.add(measurment);
      }
    }
    fieldsTable.recordOutput("Vision measurements", measurements.toArray(new VisionMeasurement[0]));
    return measurements;
  }

  public void periodic() {
    if (RobotBase.isSimulation()) {
      VisionConstants.Sim.VISION_SIM.update(PoseEstimator.getInstance().getOdometryPose());
    }
    for (VisionMeasurement mesurement : getAllResults()) {
      PoseEstimator.getInstance().addVisionMeasurement(mesurement);
    }
  }

  private static TrustLevel calculateTrustLevel(double stdFactor, int tagsUsed, double avgDistanceToCam, double ambiguity) {
    if (ambiguity == 1 || tagsUsed == 0 || avgDistanceToCam == 0)
      return new TrustLevel(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    double value = Math.pow(avgDistanceToCam, 2) / Math.pow(tagsUsed, 2) / Math.pow(1 - ambiguity, 2) * stdFactor;
    TrustLevel result = new TrustLevel(value, value);
    result = result.multiply(trustLevelMultiplier.get());
    if (tagsUsed == 1 && !RobotState.isDisabled()) result = new TrustLevel(result.xyStdDev(), Double.POSITIVE_INFINITY);
    if (RobotState.isDisabled()) result = result.multiply(preMatchTrustLevelMultiplier.get());
    return result;
  }

  @Override
  public void publishTunable(TunableTable table) {
    table.publish("Trust level multiplier", trustLevelMultiplier);
    table.publish("PreMatch trust level multiplier", preMatchTrustLevelMultiplier);
    table.publishDouble("Ambiguity threshold", ambiguityThreshold::get, ambiguityThreshold::set);
    table.publishDouble("Distance threshold", distanceThresholdMeters::get, distanceThresholdMeters::set);
  }
}
