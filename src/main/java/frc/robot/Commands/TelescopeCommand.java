package frc.robot.Commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Telescope;

public class TelescopeCommand extends CommandBase {
  private DoubleSupplier setpoint;
  private DoubleSupplier theta;
  private Telescope telescope;


  public TelescopeCommand(DoubleSupplier setpoint, DoubleSupplier theta, Telescope telescope) {
    this.setpoint = setpoint;
    this.theta = theta;
    this.telescope = telescope;
    addRequirements(telescope);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    telescope.resetTelescopeProfile(telescope.getTelescopeEncoderMeters().getAsDouble());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telescope.setTelescope(telescope.telescopeCalc(setpoint, theta));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return telescope.finish();
  }
}
