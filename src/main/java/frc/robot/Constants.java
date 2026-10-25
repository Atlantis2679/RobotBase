package frc.robot;

import static org.wpilib.units.Units.Seconds;

import org.littletonrobotics.junction.LoggedRobot;
import org.wpilib.units.measure.Time;

public class Constants {
    public static final Time ROBOT_PERIOD = Seconds.of(LoggedRobot.defaultPeriodSecs);

    public static final boolean REPLAY = false;

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
}
