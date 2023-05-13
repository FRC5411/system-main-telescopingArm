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
    telescopeMotor = Configs.NEO(telescopeMotor, 19, false);

    telescopeEncoder = Configs.relativeEncbore(telescopeEncoder, 5, 6, TelescopeConstants.METERPERROTATION);

    telescopePID = new ProfilePIDController(0, 0, 0, new TProfile.Constraints(0, 0));

    telescopeFF = new AngledElevatorFeedForward(0, 0, 0, 0);
  }

  public double telescopeCalc(DoubleSupplier setpointMeters, DoubleSupplier thetaRadians) {
    double FF = setTelescopeFF(thetaRadians);

    double PID = setTelescopePID(setpointMeters, getTelescopeEncoderMeters());

    return FF + PID;
  }

  public double setTelescopeFF(DoubleSupplier thetaRadians) {
    return telescopeFF.calculate(getProfile().velocity, getProfile().acceleration, thetaRadians.getAsDouble());
  }

  public double setTelescopePID(DoubleSupplier setpoint, DoubleSupplier measure) {
    return telescopePID.calculate(setpoint.getAsDouble(), getTelescopeEncoderMeters().getAsDouble());
  }

  public void setTelescope(double voltage) {
    telescopeMotor.setVoltage(voltage);
  }

  public void resetTelescopeProfile(double pos) {
    telescopePID.reset(pos);
  }

  public DoubleSupplier getTelescopeEncoderMeters() {
    return telescopeEncoder::getDistance;
  }

  public boolean finish() {
    return telescopePID.atGoal();
  }

  public TProfile.State getProfile() {
    return telescopePID.getSetpoint();
  }

  public DoubleSupplier getScale() {
    return () -> (getTelescopeEncoderMeters().getAsDouble() / TelescopeConstants.MIN_LENGTH);
  }

  @Override
  public void periodic() {
    Telemetry.setValue("telescope/stage1/setpoint", telescopeMotor.get());
    Telemetry.setValue("telescope/stage1/temperature", telescopeMotor.getMotorTemperature());
    Telemetry.setValue("telescope/stage1/outputVoltage", telescopeMotor.getAppliedOutput());
    Telemetry.setValue("telescope/stage1/statorCurrent", telescopeMotor.getOutputCurrent());
    Telemetry.setValue("telescope/stage1/actualPosition", Math.toDegrees(telescopeEncoder.getDistance()));
  }
}
