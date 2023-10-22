package frc.lib.refvalues;

public class IntRefValue {
    public int value;

    public IntRefValue(int value) {
        this.value = value;
    }

    public int get() {
        return value;
    }

    public void set(int value) {
        this.value = value;
    }
}
