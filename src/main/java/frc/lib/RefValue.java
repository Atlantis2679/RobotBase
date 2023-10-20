package frc.lib;

public class RefValue<T> {
    public T value;

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
