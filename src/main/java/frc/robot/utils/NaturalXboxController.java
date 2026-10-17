package frc.robot.utils;

import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.driverstation.Gamepad.Axis;

/**
 * Extension of the standard CommandXboxController with the following features:
 * - Lossless automatic deadband handling, preserving values below the deadband.
 * - Inverted Y axis for a more intuitive up-positive orientation.
 * - Squared axis values getters, providing finer control sensitivity.
 */
public class NaturalXboxController extends CommandXboxController {
    private double deadband;

    public NaturalXboxController(int port, double deadband) {
        super(port);
        this.deadband = deadband;
    }

    public NaturalXboxController(int port) {
        this(port, 0.05);
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    @Override
    public double getRightX() {
        return -1 * applyDeadband(deadband, super.getRightX());
    }

    @Override
    public double getRightY() {
        return -1 * applyDeadband(deadband, super.getRightY());
    }

    @Override
    public double getLeftX() {
        return -1 * applyDeadband(deadband, super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return -1 * applyDeadband(deadband, super.getLeftY());
    }

    @Override
    public double getLeftTrigger() {
        return applyDeadband(deadband, super.getLeftTrigger());
    }

    @Override
    public double getRightTrigger() {
        return applyDeadband(deadband, super.getRightTrigger());
    }

    public double getSquaredRightX() {
        return square(getRightX());
    }

    public double getSquaredRightY() {
        return square(getRightY());
    }

    public double getSquaredLeftX() {
        return square(getLeftX());
    }

    public double getSquaredLeftY() {
        return square(getLeftY());
    }

    public double getLeftTriggerSquared() {
        return square(getLeftTrigger());
    }

    public double getRightTriggerSquared() {
        return square(getRightTrigger());
    }

    public Trigger axisLessThan(Axis axis, double threshold) {
        return axisLessThan(axis, threshold);
    }

    public Trigger axisGreaterThan(Axis axis, double threshold) {
        return axisGreaterThan(axis, threshold);
    }

    /**
     * Applies a deadband by considering values within the deadband as the new zero.
     * Values outside the deadband are adjusted to be relative to the new range.
     *
     * @param value    The input value to apply the deadband to.
     * @param deadband The deadband range. Values within this range are considered
     *                 the new zero.
     * @return The adjusted value after applying the deadband.
     */
    public static double applyDeadband(double deadband, double value) {
        return Math.abs(value) < deadband
                ? 0
                : (value - (Math.signum(value) * deadband)) / (1 - deadband);
    }

    public static double square(double value) {
        return Math.pow(value, 2) * (value >= 0 ? 1 : -1);
    }
}
