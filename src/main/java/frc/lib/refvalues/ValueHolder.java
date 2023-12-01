package frc.lib.refvalues;

public class ValueHolder<T> {
    private T value;

    public ValueHolder(T value){
        this.value = value;
    }

    public T get() {
        return value;
    }

    public void set(T value) {
        this.value = value;
    }
}
