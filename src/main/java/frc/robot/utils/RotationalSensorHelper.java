package frc.robot.utils;

import org.wpilib.command3.Command;
import org.wpilib.system.Timer;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableTable;

import frc.robot.utils.MathUtils.Bounds;

public class RotationalSensorHelper implements ComplexTunable {
    private double measuredAngle;
    private double offset;
    private double previousAngle;
    private double currTimeSec;
    private double prevTimeSec;
    private double calculatedAngle;

    private boolean continousWrapEnabled;
    private Bounds bounds;

    public RotationalSensorHelper(double initialMeasuredAngle, double initialOffset) {
        measuredAngle = initialMeasuredAngle;
        offset = initialOffset;
        prevTimeSec = Timer.getTimestamp();
        recalculateAngle();
    }

    public RotationalSensorHelper(double initalMeasuredAngle) {
        this(initalMeasuredAngle, 0);
    }

    public void update(double measuredAngle) {
        this.measuredAngle = measuredAngle;
        prevTimeSec = currTimeSec;
        currTimeSec = Timer.getTimestamp();
        previousAngle = calculatedAngle;
        recalculateAngle();
    }

    public void recalculateAngle() {
        calculatedAngle = measuredAngle - offset;
        if (continousWrapEnabled) {
            calculatedAngle = bounds.wrap(calculatedAngle);
        }
    }

    public void resetAngle(double newAngle) {
        setOffset(measuredAngle - newAngle);
    }

    public void enableContinuousWrap(Bounds bounds) {
        continousWrapEnabled = true;
        this.bounds = bounds;
        recalculateAngle();
    }

    public void setOffset(double offset) {
        previousAngle += this.offset - offset;
        if(continousWrapEnabled)
            previousAngle = bounds.wrap(previousAngle);
        this.offset = offset;
        recalculateAngle();
    }

    public double getAngle() {
        return calculatedAngle;
    }

    public double getMeasuredAngle() {
        return measuredAngle;
    }

    public double getVelocity() {
        double deltaTime = currTimeSec - prevTimeSec;
        return (getAngle() - previousAngle) / deltaTime;
    }

    public double getOffset() {
        return offset;
    }

    @Override
    public void publishTunable(TunableTable table) {
        TunableDouble angleToReset = TunableDouble.create();
        table.publish("angleToReset", angleToReset);
        TunableCommand resetCommand = new TunableCommand(Command.noRequirements((coroutine) -> {
            resetAngle(angleToReset.get());
        }).named("ResetRotationalSensorAngle"));
        table.publish("Reset!", resetCommand);
    }
}
