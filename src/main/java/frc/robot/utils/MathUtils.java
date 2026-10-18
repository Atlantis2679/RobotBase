package frc.robot.utils;

import java.util.Arrays;

import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableTable;

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

    public static class Bounds implements ComplexTunable {
        public final TunableDouble lowerBound;
        public final TunableDouble upperBound;

        public Bounds(TunableDouble lowerBound, TunableDouble upperBound) {
            this.lowerBound = lowerBound;
            this.upperBound = upperBound;
        }

        public Bounds(double lowerBound, double upperBound) {
            this(TunableDouble.create(lowerBound), TunableDouble.create(upperBound));
        }

        public double clamp(double value) {
            return Math.clamp(value, lowerBound.get(), upperBound.get());
        }

        public boolean inBound(double value) {
            return inRange(value, lowerBound.get(), upperBound.get());
        }

        public double wrap(double value) {
            double delta = upperBound.get() - lowerBound.get();
            return ((value - lowerBound.get()) % delta + delta) % delta + lowerBound.get();
        }
 
        @Override
        public void publishTunable(TunableTable table) {
            table.publish("lowerBound", lowerBound);
            table.publish("upperBound", upperBound);
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
