package frc.lib.refvalues;

public class BooleanRefValue {
    public boolean value;

    public BooleanRefValue(boolean value) {
        this.value = value;
    }

    public boolean get() {
        return value;
    }

    public void set(boolean value) {
        this.value = value;
    }
}
