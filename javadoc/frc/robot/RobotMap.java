package frc.robot;

public class RobotMap {
    public static final String CANBUS_NAME = "rio";

    public static class Climber { 
        public static final int CLIMBER_MASTER = 0; // Drive motor - FR (Testing)
        public static final int CLIMBER_FOLLOWER = 0; // Drive motor - FL (Testing)
        public static final int LEFT_TOP_LIMIT_SWITCH = 0; // NOT REAL PORTS 
        public static final int LEFT_BOT_LIMIT_SWITCH = 1; 
        public static final int RIGHT_TOP_LIMIT_SWITCH = 2; 
        public static final int RIGHT_BOT_LIMIT_SWITCH = 3; 
    }

    public static class Intake {
        public static final int INTAKE_MOTOR = 18;
        public static final int TOP_IR = 1; 
        public static final int BOTTOM_IR = 2; 
    }

    public static class Arm {
        public static final int LEFT_MOTOR_PORT = 32;
        public static final int RIGHT_MOTOR_PORT = 14;
        public static final int ANALOG_POT_PORT = 4;
    }

    public static class Flywheel {
        public static final int LEFT_FLYWHEEL = 20; 
        public static final int RIGHT_FLYWHEEL = 16; 
    }
}
