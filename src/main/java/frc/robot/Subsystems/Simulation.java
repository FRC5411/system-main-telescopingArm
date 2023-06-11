package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// failed attemtp to set up mech 2d due to a smartdashboard glitch, ignore file
public class Simulation extends SubsystemBase {
  Mechanism2d holder;
//  MechanismLigament2d arm;
  MechanismRoot2d holderRoot;

  public Simulation() {
    holder = new Mechanism2d(2, 3);
    holderRoot = holder.getRoot("Holder", 1, 1);
  //  arm = new MechanismLigament2d("Arm", 5.0, 0.0);

 //   holderRoot.append(arm);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Simulation/holder", holder);
  }
}
