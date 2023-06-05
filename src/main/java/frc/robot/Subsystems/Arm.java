package frc.robot.Subsystems;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.Configs;
import frc.robot.Libs.ProfilePIDController;
import frc.robot.Libs.ScaledArmFeedForward;
import frc.robot.Libs.TProfile;
import frc.robot.Libs.Telemetry;
import frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  private WPI_TalonSRX armMotor;
  private DutyCycleEncoder armEncoder;
  private ProfilePIDController armPID;
  private ScaledArmFeedForward armFF;

  public Arm() {
    armMotor = Configs.SRX(armMotor, 12, false);
    armFF = new ScaledArmFeedForward(ArmConstants.KS, ArmConstants.KG, ArmConstants.KV, ArmConstants.KA);
  }

  public double armCalc(DoubleSupplier setpointRadians, DoubleSupplier scale) {
    double FF = setArmFF(scale);
    Telemetry.setValue("Arm/stage1/FF", FF);
    double PID = setArmPID(setpointRadians);
    Telemetry.setValue("Arm/stage1/PID", PID);
    return FF + PID;
  }

  public double setArmFF(DoubleSupplier scale) {
    return armFF.calculate(getProfile().position, getProfile().velocity, getProfile().acceleration, scale.getAsDouble());
  }

  public void setArm(double voltage) {
    armMotor.set(ControlMode.Position);
  }

  public DoubleSupplier getArmEncoderRadians() {
    return armEncoder::getAbsolutePosition;
  }

  public boolean finish() {
    return armPID.atGoal();
  }

  public TProfile.State getProfile() {
    return armPID.getSetpoint();
  }

  @Override
  public void periodic() {}
}
