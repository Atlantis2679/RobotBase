package frc.robot.subsystems.vision.io;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class VisionAprilTagsIO extends IOBase {
    public record VisionData(
        Time timestamp,
        Pose3d robotPose,
        Distance avgDistanceToCam,
        double ambiguity,
        int tagsUsed) {}

    public final BooleanSupplier isConnected = fields.addBoolean("isConnected", this::getIsConnected);
    public final Supplier<VisionData[]> visionData = fields.addRecordArray("visionData", this::visionData, new VisionData[0]); 

    protected VisionAprilTagsIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public abstract CameraConfig getCameraConfig();

    protected abstract boolean getIsConnected();

    protected abstract VisionData[] visionData();
}
