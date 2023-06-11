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


  public double[] armKinematics(double x, double y) {
    double armX = x - TelescopeConstants.XOFFSET;
    double armY = y - TelescopeConstants.YOFFSET;

    double thetaRadians = Math.atan2(armY, armX);
    double magnitude = Math.hypot(armX, armY);

    double[] polarVals = {magnitude, thetaRadians};

    if(magnitude > TelescopeConstants.MAX_LENGTH) {
      polarVals[0] = TelescopeConstants.MAX_LENGTH;
    }

    return polarVals;
  }

  // x,  y version
  public ParallelCommandGroup goToPos(double x, double y) {
    double[] polarVals = armKinematics(x, y);
    return goToPosPolar(polarVals[0], Math.toDegrees(polarVals[1]));
  }

  // Uses degrees as its better for human interaction, while radians is better for the lower level
  // Parallel Command Group is used to run both the arm and telescope at the same time, but my Feedforwards
  // May not account for higher level dynamics, so if control prove difficults switch to a sequential command group
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
