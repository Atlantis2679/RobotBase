package frc.lib.tuneables;

public enum SendableType {
    NONE(null),
    LIST("LW Subsystem"),
    PID("PIDController");

    private String stringType;

    private SendableType(String stringType) {
        this.stringType = stringType;
    }

    public String getStringType() {
        return stringType;
    }
}
