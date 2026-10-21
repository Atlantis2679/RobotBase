package frc.robot.utils;

import java.util.Arrays;

import org.wpilib.math.util.MathUtil;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;

public class MathUtils {
    public static class CosineWaveFollower implements ComplexTunable {
        public double min, max, speed, timestamp;

        public CosineWaveFollower(double min, double max, double speed) {
            this.min = min;
            this.max = max;
            this.speed = speed;
            this.timestamp = 0;
        }

        public CosineWaveFollower(double min, double max) {
            this(min, max, Math.PI / 180);
        }

        public double getNext() {
            timestamp += speed;
            return cosineWave(timestamp, min, max);
        }

        public static double cosineWave(double timestamp, double min, double max) {
            double average = (max + min) / 2;
            double delta = (max - min) / 2;
            return average + delta * Math.cos(timestamp);
        }

        @Override
        public void publishTunable(TunableTable table) {
            table.publishDouble("min",  () -> this.min, (min) -> this.min = min);
            table.publishDouble("max",  () -> this.max, (max) -> this.max = max);
            table.publishDouble("speed", () -> this.speed, (speed) -> this.speed = speed);
        }
    }

    public static final class Bounds<M extends Measure<?>> implements ComplexTunable {
        public final TunableMeasure<M> lowerBound;
        public final TunableMeasure<M> upperBound;

        private Bounds(TunableMeasure<M> lowerBound, TunableMeasure<M> upperBound) {
            this.lowerBound = lowerBound;
            this.upperBound = upperBound;
        }

        public static <U extends Unit, M extends Measure<U>> Bounds<M> of(M lower, M upper) {
            U unit = lower.unit();
            return new Bounds<>(TunableMeasure.of(lower, unit), TunableMeasure.of(upper, unit));
        }

        public M clamp(M value) {
            M lowerBound = this.lowerBound.get();
            M upperBound = this.upperBound.get();
            if (value.baseUnitMagnitude() < lowerBound.baseUnitMagnitude()) return lowerBound;
            if (value.baseUnitMagnitude() > upperBound.baseUnitMagnitude()) return upperBound;
            return value;
        }

        public boolean contains(M value) {
            return value.baseUnitMagnitude() >= lowerBound.get().baseUnitMagnitude() && value.baseUnitMagnitude() <= upperBound.get().baseUnitMagnitude();
        }

        @SuppressWarnings("unchecked")
        public M wrap(M value) {
            double wrapped = MathUtil.inputModulus(
                value.baseUnitMagnitude(),
                lowerBound.get().baseUnitMagnitude(),
                upperBound.get().baseUnitMagnitude());
            return (M) value.unit().ofBaseUnits(wrapped);
        }

        @Override
        public void publishTunable(TunableTable table) {
            table.publish("lowerBound", lowerBound.tunable());
            table.publish("upperBound", upperBound.tunable());
        }
    }

    public static double avg(double... values) {
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.length;
    }

    public static double median(double... arr) {
        int n = arr.length;
        Arrays.sort(arr);
        if (n % 2 != 0) {
          return arr[n / 2];
        }
        return avg(arr[n / 2], arr[n / 2 + 1]);
    }

    public static boolean inRange(double value, double lowerBound, double upperBound) {
        return value >= lowerBound && value <= upperBound;
    }
}
