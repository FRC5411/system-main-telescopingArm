package frc.robot.Subsystems;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.TelescopeCommand;
import frc.robot.Constants.TelescopeConstants;

public class ArmManager extends SubsystemBase {
  Telescope telescope;
  Arm arm;
  
  public ArmManager(Arm arm, Telescope telescope) {
    this.arm = arm;
    this.telescope = telescope;
  }


  public double[] armKinematicss(double x, double y) {
    double armX = x - TelescopeConstants.XOFFSET;
    double armY = y - TelescopeConstants.YOFFSET;

    double theta = Math.atan2(armY, armX);
    double magnitude = Math.hypot(armX, armY);

    double[] polarVals = {magnitude, theta};

    if(magnitude > TelescopeConstants.MAX_LENGTH) {
      polarVals[0] = TelescopeConstants.MAX_LENGTH;
    }

    return polarVals;
  }

  public ParallelCommandGroup goToPos(double x, double y) {
    double[] polarVals = armKinematicss(x, y);
    return new ParallelCommandGroup(new ArmCommand(() -> polarVals[0],
    telescope.getScale(),
    arm),
    new TelescopeCommand(() -> polarVals[1],
    arm.getArmEncoderRadians(),
    telescope));
  }


  public ParallelCommandGroup goToPosPolar(double magnitude, double thetaDegrees) {
    return new ParallelCommandGroup(new ArmCommand(() -> Math.toRadians(thetaDegrees),
    telescope.getScale(),
    arm),
    new TelescopeCommand(() -> magnitude,
    arm.getArmEncoderRadians(),
    telescope));
  }

  @Override
  public void periodic() {}
}
