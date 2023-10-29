package frc.lib.refvalues;

public class DoubleRefValue {
    private double value;

    public DoubleRefValue(double value) {
        this.value = value;
    }

    public double get() {
        return value;
    }

    public void set(double value) {
        this.value = value;
    }
}
