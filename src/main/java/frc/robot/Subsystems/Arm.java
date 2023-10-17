package frc.robot.Subsystems;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.Configs;
import frc.robot.Libs.ProfilePIDController;
import frc.robot.Libs.ScaledArmFeedForward;
import frc.robot.Libs.TProfile;
import frc.robot.Libs.TProfile.Constraints;
import frc.robot.Libs.Telemetry;
import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor;
  private DutyCycleEncoder armEncoder;
  private ProfilePIDController armPID;
  private ScaledArmFeedForward armFF;


  // All angles in terms of Radians
  public Arm() {
    armMotor = Configs.NEO(armMotor, 12, false);
    armEncoder = Configs.AbsEncbore(armEncoder, 0, 2 * Math.PI);

    armPID = new ProfilePIDController(
      ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, 
      new Constraints(ArmConstants.kMaxVelocity, ArmConstants.kMaxAcceleration));

    armPID.setTolerance(0.0);
    armPID.enableContinuousInput(-Math.PI, Math.PI);

    armFF = new ScaledArmFeedForward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
  }

  // Returns calculations for FF and PID
  public double armCalc(DoubleSupplier setpointRadians, DoubleSupplier scale) {
    double FF = setArmFF(scale);
    Telemetry.setValue("Arm/stage1/FF", FF);
    double PID = setArmPID(setpointRadians);
    Telemetry.setValue("Arm/stage1/PID", PID);
    return FF + PID;
  }

  public double setArmFF(DoubleSupplier scale) {
    // m - mass, g - gravity, theta - angle, l - length
    // Due to kg Formula being m * g * cos(theta) * l/2, the l variable is dynamic along with the theta variable
    // Due to the nature of a telescoping arm, the l variable is dynamic
    return armFF.calculate(
      getProfile().position, 
      getProfile().velocity, 
      getProfile().acceleration, 
      scale.getAsDouble());
  }

  public double setArmPID(DoubleSupplier setpoint) {
    return armPID.calculate(
      setpoint.getAsDouble(), 
      getArmEncoderRadians().getAsDouble());
  }

  public void resetArmProfile(double pos) {
    armPID.reset(pos);
  }

  public void setArm(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public DoubleSupplier getArmEncoderRadians() {
    return armEncoder::getAbsolutePosition;
  }

  public boolean armAtGoal() {
    return armPID.atGoal();
  }

  public TProfile.State getProfile() {
    return armPID.getSetpoint();
  }

  @Override
  public void periodic() {
    // Telemetry
    Telemetry.setValue("Arm/setpoint", armMotor.get());
    Telemetry.setValue("Arm/temperature", armMotor.getMotorTemperature());
    Telemetry.setValue("Arm/outputVoltage", armMotor.getAppliedOutput());
    Telemetry.setValue("Arm/statorCurrent", armMotor.getOutputCurrent());
    Telemetry.setValue("Arm/actualPosition", Math.toDegrees(armEncoder.getAbsolutePosition()));
  }
}
