package frc.robot.subsystems.poseestimation;

import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;

public class TunableTrustLevel implements ComplexTunable {
  private TrustLevel trustLevel;

  public TunableTrustLevel(TrustLevel trustLevel) {
    this.trustLevel = trustLevel;
  }

  public TrustLevel get() {
    return trustLevel;
  }

  public void set(TrustLevel trustLevel) {
    this.trustLevel = trustLevel;
  }

  @Override
  public void publishTunable(TunableTable table) {
    table.publishDouble("xyStdDev", () -> trustLevel.xyStdDev(), (xyStdDev) -> set(new TrustLevel(xyStdDev, trustLevel.rotationStdDev())));
    table.publishDouble("rotationStdDev", () -> trustLevel.rotationStdDev(), (rotationStdDev) -> set(new TrustLevel(trustLevel.xyStdDev(), rotationStdDev)));
  }
}
