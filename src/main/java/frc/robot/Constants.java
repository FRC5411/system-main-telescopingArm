package frc.robot;

public final class Constants {
    public class TelescopeConstants {
        // Physical arm details
        public static final double kMinLength = 1;
        public static final double kMaxLength = 10;

        public static final double kGearRatio = 0;
        public static final double kMetersPerRotation = 1;
        public static final double kConversionFactorMeters = kGearRatio * kMetersPerRotation;

        public static final double XOFFSET  = 0;
        public static final double YOFFSET = 0;


        // ALL CONSTANTS SUCK, DO NOT USE THEM ONLY FOR TESTING PURPOSES
        // PID constants
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 1;

        public static final double kS = 1;
        public static final double kG = 1;
        public static final double kV = 1;
        public static final double kA = 1;

        // Constraints
        public static final double kMaxVelocity = 1;
        public static final double kMaxAcceleration = 1;
    }

    public class ArmConstants {
        // Physical arm details       
        public static final double kMinDegrees = 1; 
        public static final double kMaxDegrees = 10;

        public static final double kGearRatio = 0; 

        // ALL CONSTANTS SUCK DO NOT USE THEM ONLY FOR TESTING PURPOSES
        // PID constants
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 1;

        // Feedforward constants        
        public static final double kS = 1;
        public static final double kG = 1;
        public static final double kV = 1;
        public static final double kA = 1;

        // Constraints
        public static final double kMaxVelocity = 1;
        public static final double kMaxAcceleration = 1;
    }

}
