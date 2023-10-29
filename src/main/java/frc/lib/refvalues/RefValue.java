package frc.lib.refvalues;

public class RefValue<T> {
    private T value;

    public RefValue(T value){
        this.value = value;
    }

    public T get() {
        return value;
    }

    public void set(T value) {
        this.value = value;
    }
}
