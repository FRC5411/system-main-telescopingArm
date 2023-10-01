package frc.robot.Libs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class Configs {

    public static CANSparkMax NEO550(CANSparkMax motor, int deviceID, boolean inverted) {
        motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(20);
        motor.setSecondaryCurrentLimit(20);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        return motor;
    }

    public static CANSparkMax NEO(CANSparkMax motor, int deviceID, boolean inverted) {
        motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.clearFaults();
        motor.setSmartCurrentLimit(40);
        motor.setSecondaryCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        return motor;
    }

    public static DutyCycleEncoder AbsEncbore(DutyCycleEncoder encoder, int port, double conversionFactor) {
        encoder = new DutyCycleEncoder(port);
        encoder.setDistancePerRotation(conversionFactor);
        return encoder;
    }

    public static Encoder relativeEncbore(Encoder encoder, int port, int port2, double conversionFactor) {
        encoder = new Encoder(port, port2);
        encoder.setDistancePerPulse(conversionFactor);
        return encoder;
    }
}