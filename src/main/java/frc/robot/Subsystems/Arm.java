package frc.robot.Subsystems;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.Configs;
import frc.robot.Libs.ProfilePIDController;
import frc.robot.Libs.ScaledArmFeedForward;
import frc.robot.Libs.TProfile;
import frc.robot.Libs.Telemetry;
import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor;
  private DutyCycleEncoder armEncoder;
  private ProfilePIDController armPID;
  private ScaledArmFeedForward armFF;


  // All angles in terms of Radians
  public Arm() {
    // Configurations to set up the motor and encoder from the config file
    armMotor = Configs.NEO(armMotor, 12, false);
    armEncoder = Configs.AbsEncbore(armEncoder, 0, 2*Math.PI);

    armPID = new ProfilePIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD, 
    new TProfile.Constraints(ArmConstants.MAXVELOCITY, ArmConstants.MAXACCELERATION));

    armPID.setTolerance(0.1);
    armPID.enableContinuousInput(-Math.PI, Math.PI);

    armFF = new ScaledArmFeedForward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV, ArmConstants.KA);
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
    return armFF.calculate(getProfile().position, getProfile().velocity, getProfile().acceleration, scale.getAsDouble());
  }

  public double setArmPID(DoubleSupplier setpoint) {
    return armPID.calculate(setpoint.getAsDouble(), getArmEncoderRadians().getAsDouble());
  }

  // Rested profiles from the current position as the ProfilePID always starts its setpoints from 0
  public void resetArmProfile(double pos) {
    armPID.reset(pos);
  }

  public void setArm(double voltage) {
    armMotor.setVoltage(voltage);
  }

  // For Arm FF and elevator FF, to account for gravity at angles
  public DoubleSupplier getArmEncoderRadians() {
    return armEncoder::getAbsolutePosition;
  }

  // Checks if the arm is at the goal for commands
  public boolean finish() {
    return armPID.atGoal();
  }

  // Gets profile values for FF
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
