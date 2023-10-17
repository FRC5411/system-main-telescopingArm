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
    // Uses configurations to set up the motor and encoder fromt he config file
    telescopeMotor = Configs.NEO(telescopeMotor, 19, false);

    telescopeEncoder = Configs.relativeEncbore(telescopeEncoder, 5, 6, TelescopeConstants.kMetersPerRotation);

    telescopePID = new ProfilePIDController(TelescopeConstants.kP, TelescopeConstants.kI, TelescopeConstants.kD, 
    new TProfile.Constraints(TelescopeConstants.kMaxVelocity, TelescopeConstants.kMaxAcceleration));

    telescopeFF = new AngledElevatorFeedForward(
      TelescopeConstants.kS, TelescopeConstants.kG, TelescopeConstants.kV, TelescopeConstants.kA);
  }


  // Puts it all together
  public double telescopeCalc(DoubleSupplier setpointMeters, DoubleSupplier thetaRadians) {
    double FF = setTelescopeFF(thetaRadians);

    double PID = setTelescopePID(setpointMeters, getTelescopeEncoderMeters());

    return FF + PID;
  }

  // return FF in voltage
  public double setTelescopeFF(DoubleSupplier thetaRadians) {
    // Differs from normal Elevator FF equation, as the equation is a special case of m * g * sin(0) = m * g
    // We need a more general version of the equation, so we use m * g * sin(theta) as due to the nature
    // Of a telescoping the elevator angle changes
    return telescopeFF.calculate(getProfile().velocity, getProfile().acceleration, thetaRadians.getAsDouble());
  }

  // Return PID calculation in voltage
  public double setTelescopePID(DoubleSupplier setpoint, DoubleSupplier measure) {
    return telescopePID.calculate(setpoint.getAsDouble(), getTelescopeEncoderMeters().getAsDouble());
  }

  // Telescope voltage
  public void setTelescope(double voltage) {
    telescopeMotor.setVoltage(voltage);
  }

  // Rested profiles from the current position as the ProfilePID always starts is setpoints from 0
  public void resetTelescopeProfile(double pos) {
    telescopePID.reset(pos);
  }

  // Used to get the encoder value in meters
  public DoubleSupplier getTelescopeEncoderMeters() {
    return telescopeEncoder::getDistance;
  }

  // Used to end the command when the telescope is at the setpoint, 
  // for a second them overshoots, but the command fnishes
  public boolean finish() {
    return telescopePID.atGoal();
  }

  // Used to get the setpoint of the Profile for feedforwards
  public TProfile.State getProfile() {
    return telescopePID.getSetpoint();
  }

  // Used to scale up the kg value in the feedforward for the Arm subsystem because physics
  public DoubleSupplier getScale() {
    return () -> (getTelescopeEncoderMeters().getAsDouble() / TelescopeConstants.kMinLength);
  }

  @Override
  public void periodic() {
    // Telemetry
    Telemetry.setValue("telescope/setpoint", telescopeMotor.get());
    Telemetry.setValue("telescope/temperature", telescopeMotor.getMotorTemperature());
    Telemetry.setValue("telescope/outputVoltage", telescopeMotor.getAppliedOutput());
    Telemetry.setValue("telescope/statorCurrent", telescopeMotor.getOutputCurrent());
    Telemetry.setValue("telescope/actualPosition", Math.toDegrees(telescopeEncoder.getDistance()));
  }
}
