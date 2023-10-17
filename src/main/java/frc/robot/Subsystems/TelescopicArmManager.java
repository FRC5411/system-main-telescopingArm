package frc.robot.Subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.Libs.Telemetry;

public class TelescopicArmManager extends SubsystemBase {
  private Telescope telescope;
  private Arm arm;
  private double armSetpointRadians;
  private double telescopeSetpointMeters;
  
  public TelescopicArmManager(Arm arm, Telescope telescope) {
    this.arm = arm;
    this.telescope = telescope;

    armSetpointRadians = this.arm.getArmEncoderRadians().getAsDouble();
    telescopeSetpointMeters = this.telescope.getTelescopeEncoderMeters().getAsDouble();
  }

  public double[] telescopeArmKinematics(double x, double y) {
    double armX = x - TelescopeConstants.XOFFSET;
    double armY = y - TelescopeConstants.YOFFSET;

    double thetaRadians = Math.atan2(armY, armX);
    double magnitude = Math.hypot(armX, armY);

    double[] polarVals = {magnitude, thetaRadians};

    return polarVals;
  }

  public Command goToPos(double x, double y) {
    double[] polarVals = telescopeArmKinematics( x, y );
    return goToPosPolar( polarVals[0], Math.toDegrees( polarVals[1] ) );
  }

  public Command goToPosPolar(double magnitude, double thetaDegrees) {
    MathUtil.clamp( magnitude, TelescopeConstants.kMinLength, TelescopeConstants.kMaxLength );
    MathUtil.clamp( thetaDegrees, ArmConstants.kMinDegrees, ArmConstants.kMaxDegrees );

    return new InstantCommand(
      () -> { 
        armSetpointRadians = Math.toRadians(thetaDegrees);
        telescopeSetpointMeters = magnitude;
       }, this);
  }

  public double getArmSetpointRadians() {
    return armSetpointRadians;
  }

  public double getTelescopeSetpointMeters() {
    return telescopeSetpointMeters;
  }

  @Override
  public void periodic() {
    Telemetry.setValue("Arm/Setpoint", armSetpointRadians);
    Telemetry.setValue("Telescope/Setpoint", telescopeSetpointMeters);
  }

  @Override
  public void simulationPeriodic() {}
}
