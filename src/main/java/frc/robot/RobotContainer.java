package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.TelescopeCommand;
import frc.robot.Constants.TelescopeConstants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.ArmManager;
import frc.robot.Subsystems.Telescope;

public class RobotContainer {
  private final Arm arm;
  private final Telescope telescope;
  private final ArmManager armManager;
  private final CommandXboxController mController;

  public RobotContainer() {
    arm = new Arm();
    telescope = new Telescope();
    armManager = new ArmManager(arm, telescope);

    mController = new CommandXboxController(0);

    // arm.setDefaultCommand(new ArmCommand(arm.getArmEncoderRadians(),
    // telescope.getScale(), 
    // arm));

    // telescope.setDefaultCommand(new TelescopeCommand(telescope.getTelescopeEncoderMeters(),
    // arm.getArmEncoderRadians(),
    // telescope));

    configureBindings();
  }

  private void configureBindings() {
    // click(mController.y(), armManager.goToPosPolar(TelescopeConstants.MAX_LENGTH, 90.0));
    // click(mController.b(), armManager.goToPosPolar(TelescopeConstants.MAX_LENGTH, 0));
    // click(mController.x(), armManager.goToPosPolar(TelescopeConstants.MAX_LENGTH, 180.0));
    // click(mController.a(), armManager.goToPosPolar(TelescopeConstants.MIN_LENGTH, 270.0));
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
