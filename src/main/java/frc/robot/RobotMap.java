package frc.robot;

import org.wpilib.hardware.bus.CANPort;

public final class RobotMap {
    public static final class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class CANBUS {
        public static record CANAddress(CANPort port, int id) {}

        public static record ModuleCanAddresses(CANAddress drive, CANAddress turn, CANAddress canCoder) {}

        public static final ModuleCanAddresses MODULE_FL = new ModuleCanAddresses(
            new CANAddress(CANPort.CAN_D0, 0),
            new CANAddress(CANPort.CAN_D0, 0),
            new CANAddress(CANPort.CAN_D0, 0));

        public static final ModuleCanAddresses MODULE_FR = new ModuleCanAddresses(
            new CANAddress(CANPort.CAN_D0, 0),
            new CANAddress(CANPort.CAN_D0, 0),
            new CANAddress(CANPort.CAN_D0, 0));

        public static final ModuleCanAddresses MODULE_BL = new ModuleCanAddresses(
            new CANAddress(CANPort.CAN_D0, 0),
            new CANAddress(CANPort.CAN_D0, 0),
            new CANAddress(CANPort.CAN_D0, 0));
            
        public static final ModuleCanAddresses MODULE_BR = new ModuleCanAddresses(
            new CANAddress(CANPort.CAN_D0, 0),
            new CANAddress(CANPort.CAN_D0, 0),
            new CANAddress(CANPort.CAN_D0, 0));

        public static final CANAddress CAN_AND_GYRO_ID = new CANAddress(CANPort.CAN_D0, 0);
    }

    public static final class DIO {
    }
}
