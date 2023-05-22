package frc.robot.Commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Libs.Telemetry;
import frc.robot.Subsystems.Arm;

public class ArmCommand extends CommandBase {
  private DoubleSupplier setpointRadians;
  private DoubleSupplier scale;
  private Arm arm;
  private double temp;


  public ArmCommand(DoubleSupplier setpointRadians, DoubleSupplier scale, Arm arm) {
    this.setpointRadians = setpointRadians;
    this.scale = scale;
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.resetArmProfile(arm.getArmEncoderRadians().getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    temp = arm.armCalc(setpointRadians, scale);
    arm.setArm(temp);
    Telemetry.setValue("armCalc", temp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.finish();
  }
}
