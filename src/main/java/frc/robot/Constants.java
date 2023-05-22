package frc.robot;

public final class Constants {
    public class TelescopeConstants {
        // Physical arm details
        public static final double GEAR_RATIO = 0;
        public static final double MAX_LENGTH = 10;
        public static final double MIN_LENGTH = 1;
        public static final double METERPERROTATION = 1;
        public static final double ConversionFactorMeters = GEAR_RATIO * METERPERROTATION;
        public static final double XOFFSET  = 0;
        public static final double YOFFSET = 0;

        // PID constants
        public static final double KP = 1;
        public static final double KI = 0;
        public static final double KD = 1;

        // Feedforward constants
        public static final double KS = 1;
        public static final double KG = 1;
        public static final double KV = 1;
        public static final double KA = 1;

        // Constraints
        public static final double MAXVELOCITY = 1;
        public static final double MAXACCELERATION = 1;
    }

    public class ArmConstants {
        // Physical arm details
        public static final double GEAR_RATIO = 0;        

        // PID constants
        public static final double KP = 1;
        public static final double KI = 0;
        public static final double KD = 1;

        // Feedforward constants        
        public static final double KS = 1;
        public static final double KG = 1;
        public static final double KV = 1;
        public static final double KA = 1;

        // Constraints
        public static final double MAXVELOCITY = 1;
        public static final double MAXACCELERATION = 1;
    }

}
