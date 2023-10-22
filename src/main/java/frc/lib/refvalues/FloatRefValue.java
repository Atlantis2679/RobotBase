package frc.lib.refvalues;

public class FloatRefValue {
    public float value;

    public FloatRefValue(float value) {
        this.value = value;
    }

    public float get() {
        return value;
    }

    public void set(float value) {
        this.value = value;
    }
}
