package frc.lib.refvalues;

public class LongRefValue {
    public long value;

    public LongRefValue(long value) {
        this.value = value;
    }

    public long get() {
        return value;
    }

    public void set(long value) {
        this.value = value;
    }
}
