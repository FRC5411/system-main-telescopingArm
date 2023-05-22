// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
