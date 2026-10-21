package frc.robot.utils;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.system.Timer;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;

import frc.robot.utils.MathUtils.Bounds;

public class RotationalSensorHelper implements ComplexTunable {
    private Angle measuredAngle;
    private Angle offset;
    private Angle previousAngle;
    private Time currTime;
    private Time prevTime;
    private Angle calculatedAngle;

    private boolean continousWrapEnabled;
    private Bounds<Angle> wrapBounds;

    public RotationalSensorHelper(Angle initialMeasuredAngle, Angle initialOffset) {
        measuredAngle = initialMeasuredAngle;
        offset = initialOffset;
        prevTime = Seconds.of(Timer.getTimestamp());
        currTime = prevTime;
        recalculateAngle();
        previousAngle = calculatedAngle;
    }

    public RotationalSensorHelper(Angle initalMeasuredAngle) {
        this(initalMeasuredAngle, Degrees.zero());
    }

    public void update(Angle measuredAngle) {
        this.measuredAngle = measuredAngle;
        prevTime = currTime;
        currTime = Seconds.of(Timer.getTimestamp());
        previousAngle = calculatedAngle;
        recalculateAngle();
    }

    public void recalculateAngle() {
        calculatedAngle = measuredAngle.minus(offset);
        if (continousWrapEnabled) {
            calculatedAngle = wrapBounds.wrap(calculatedAngle);
        }
    }

    public void resetAngle(Angle newAngle) {
        setOffset(measuredAngle.minus(newAngle));
    }

    public void enableContinuousWrap(Bounds<Angle> wrapBounds) {
        continousWrapEnabled = true;
        this.wrapBounds = wrapBounds;
        recalculateAngle();
    }

    public void setOffset(Angle offset) {
        previousAngle = previousAngle.plus(this.offset).minus(offset);
        if(continousWrapEnabled)
            previousAngle = wrapBounds.wrap(previousAngle);
        this.offset = offset;
        recalculateAngle();
    }

    public Angle getAngle() {
        return calculatedAngle;
    }

    public Angle getMeasuredAngle() {
        return measuredAngle;
    }

    public AngularVelocity getVelocity() {
        Time dt = currTime.minus(prevTime);
        return getAngle().minus(previousAngle).div(dt);
    }

    public Angle getOffset() {
        return offset;
    }

    @Override
    public void publishTunable(TunableTable table) {
        table.publish("Wrap Bounds", wrapBounds);
        table.publish("Offset", TunableMeasure.bind(this::getOffset, this::setOffset, Degrees));
        table.publish("Angle", TunableMeasure.bind(this::getAngle, this::resetAngle, Degrees));
    }
}
