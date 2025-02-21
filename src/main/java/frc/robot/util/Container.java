package frc.robot.util;

/**
 * A container that allows internal mutability. Used for mutating captured values.
 */
public class Container<T> {
    public T value;

    public Container(T value) {
        this.value = value;
    }
}
