package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.TelescopeCommand;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.TelescopicArmManager;
import frc.robot.Subsystems.Telescope;

public class RobotContainer {
  private final Arm arm;
  private final Telescope telescope;
  private final TelescopicArmManager telescopingArmManager;
  private final CommandXboxController mController;

  public RobotContainer() {
    arm = new Arm();
    telescope = new Telescope();
    telescopingArmManager = new TelescopicArmManager(arm, telescope);

    mController = new CommandXboxController(0);

    arm.setDefaultCommand(
      new ArmCommand(
        () -> telescopingArmManager.getArmSetpointRadians(),
        telescope.getScale(), 
        arm));

    telescope.setDefaultCommand(
      new TelescopeCommand(
        () -> telescopingArmManager.getTelescopeSetpointMeters(),
        arm.getArmEncoderRadians(),
        telescope));

    configureBindings();
  }

  private void configureBindings() {
    // If the arm is built make sure that the arm doesnt breakt the robot and that the encoders are set up properply
    click(mController.y(), telescopingArmManager.goToPosPolar(TelescopeConstants.kMaxLength, 90.0));
    click(mController.b(), telescopingArmManager.goToPosPolar(TelescopeConstants.kMaxLength, 0));
    click(mController.x(), telescopingArmManager.goToPosPolar(TelescopeConstants.kMaxLength, 180.0));
    click(mController.a(), telescopingArmManager.goToPosPolar(TelescopeConstants.kMinLength, 270.0));
  }

  public void click(Trigger button, Command command, Command command2) {
    button.onTrue(command);
    button.onFalse(command2);
  }

  public void click(Trigger button, Command command) {
    button.onTrue(command);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
