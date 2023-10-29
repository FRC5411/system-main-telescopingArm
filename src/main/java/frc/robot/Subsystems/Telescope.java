package frc.robot.Subsystems;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.AngledElevatorFeedForward;
import frc.robot.Libs.Configs;
import frc.robot.Libs.ProfilePIDController;
import frc.robot.Libs.TProfile;
import frc.robot.Libs.Telemetry;
import frc.robot.Constants.*;

public class Telescope extends SubsystemBase {
  private CANSparkMax telescopeMotor;
  private Encoder telescopeEncoder;
  private ProfilePIDController telescopePID;
  private AngledElevatorFeedForward telescopeFF;

  public Telescope() {
    // Uses configurations to set up the motor and encoder fromt the config file
    telescopeMotor = Configs.NEO(telescopeMotor, 19, false);

    // Sets up the encoder by specifying the ports and the conversion factor
    telescopeEncoder = Configs.relativeEncbore(telescopeEncoder, 5, 6, TelescopeConstants.kMetersPerRotation);

    // Creates a new ProfilePIDController for the telescope
    telescopePID = new ProfilePIDController(TelescopeConstants.kP, TelescopeConstants.kI, TelescopeConstants.kD, 
    new TProfile.Constraints(TelescopeConstants.kMaxVelocity, TelescopeConstants.kMaxAcceleration));

    // Creates a new Feed Forward for the telescope
    telescopeFF = new AngledElevatorFeedForward(
      TelescopeConstants.kS, TelescopeConstants.kG, TelescopeConstants.kV, TelescopeConstants.kA);
  }

  // Theta is the symbol in math to denote radians. Radians are based off of the unit circle.
  public double telescopeCalc(DoubleSupplier setpointMeters, DoubleSupplier thetaRadians) {
    // Sets the FF values for the elevator based on the current 
    // position of the encoder
    double FF = setTelescopeFF(thetaRadians);
    // Sets FF on telemetry
    Telemetry.setValue("Telescope/FF", FF);
    // Creates the PID for the telescope
    double PID = setTelescopePID(setpointMeters);
    // Puts PID on telemetry
    Telemetry.setValue("Telescope/PID", PID);

    // Returns a value for you to put into your motor 
    // To give it a velocity to get to your setpoint
    return FF + PID;
  }

  public double setTelescopeFF(DoubleSupplier thetaRadians) {
    /* Differs from normal Elevator FF equation, as the equation is a special case of m * g * sin(0) = m * g
     We need a more general version of the equation, so we use m * g * sin(theta) as due to the nature
     Of a telescoping the elevator angle changes */
    return telescopeFF.calculate(getProfile().velocity, getProfile().acceleration, thetaRadians.getAsDouble());
  }

    // Calulates the PID, take DoubleSupplier measure out
  public double setTelescopePID(DoubleSupplier setpoint) {
    return telescopePID.calculate(getTelescopeEncoderMeters().getAsDouble(), setpoint.getAsDouble());
  }

  // Gives the voltage to the motors.
  public void setTelescope(double voltage) {
    telescopeMotor.setVoltage(voltage);
  }

  // Resets the telescope to the current position
  // of the encoder
  public void resetTelescopeProfile(double pos) {
    telescopePID.reset(pos);
  }

  // Gets how much the encoder has ran
  public DoubleSupplier getTelescopeEncoderMeters() {
    return telescopeEncoder::getDistance;
  }

  // Returns true if the error is within
  // the specified tolerance
  public boolean atGoal() {
    return telescopePID.atGoal();
  }

  // Gets the setpoint of the telescope PID
  public TProfile.State getProfile() {
    return telescopePID.getSetpoint();
  }

  // kMinLength is currently a placeholder,
  public DoubleSupplier getScale() {
    return () -> (getTelescopeEncoderMeters().getAsDouble() / TelescopeConstants.kMinLength);
  }

  @Override
  public void periodic() {
    Telemetry.setValue("Telescope/temperature", telescopeMotor.getMotorTemperature());
    Telemetry.setValue("Telescope/outputVoltage", telescopeMotor.getAppliedOutput());
    Telemetry.setValue("Telescope/statorCurrent", telescopeMotor.getOutputCurrent());
    Telemetry.setValue("Telescope/actualPosition", Math.toDegrees(telescopeEncoder.getDistance()));
  }
}
