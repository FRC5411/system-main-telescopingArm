package frc.robot.Commands;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Libs.Telemetry;
import frc.robot.Subsystems.Telescope;

public class TelescopeCommand extends CommandBase {
  private DoubleSupplier setpointMeters;
  private DoubleSupplier thetaRadians;
  private Telescope telescope;


  public TelescopeCommand(DoubleSupplier setpointMeters, DoubleSupplier thetaRadians, Telescope telescope) {
    this.setpointMeters = setpointMeters;
    this.thetaRadians = thetaRadians;
    this.telescope = telescope;
    addRequirements(telescope);
  }

  @Override
  public void initialize() {
    telescope.resetTelescopeProfile(telescope.getTelescopeEncoderMeters().getAsDouble());
  }

  @Override
  public void execute() {
    double temp = telescope.telescopeCalc(setpointMeters, thetaRadians);
    telescope.setTelescope(temp);
    Telemetry.setValue("temp", temp);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
