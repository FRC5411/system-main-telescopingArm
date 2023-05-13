// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.Configs;
import frc.robot.Libs.ProfilePIDController;
import frc.robot.Libs.ScaledArmFeedForward;
import frc.robot.Libs.TProfile;
import frc.robot.Libs.Telemetry;

public class Arm extends SubsystemBase {
  private CANSparkMax armMotor;
  private DutyCycleEncoder armEncoder;
  private ProfilePIDController armPID;
  private ScaledArmFeedForward armFF;

  public Arm() {
    armMotor = Configs.NEO(armMotor, 12, false);
    armEncoder = Configs.AbsEncbore(armEncoder, 0, 2*Math.PI);

    armPID = new ProfilePIDController(0, 0, 0, new TProfile.Constraints(0, 0));

    armFF = new ScaledArmFeedForward(0, 0, 0, 0);
  }

  public double armCalc(DoubleSupplier setpointRadians, DoubleSupplier scale) {
    double FF = setArmFF(scale);

    double PID = setArmPID(setpointRadians);

    return FF + PID;
  }

  public double setArmFF(DoubleSupplier scale) {
    return armFF.calculate(getProfile().position, getProfile().velocity, getProfile().acceleration, scale.getAsDouble());
  }

  public double setArmPID(DoubleSupplier setpoint) {
    return armPID.calculate(setpoint.getAsDouble(), getArmEncoderRadians().getAsDouble());
  }

  public void resetArmProfile(double pos) {
    armPID.reset(pos);
  }
  public void setArm(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public DoubleSupplier getArmEncoderRadians() {
    return armEncoder::getAbsolutePosition;
  }

  public boolean finish() {
    return armPID.atGoal();
  }

  public TProfile.State getProfile() {
    return armPID.getSetpoint();
  }

  @Override
  public void periodic() {
    Telemetry.setValue("Arm/stage1/setpoint", armMotor.get());
    Telemetry.setValue("Arm/stage1/temperature", armMotor.getMotorTemperature());
    Telemetry.setValue("Arm/stage1/outputVoltage", armMotor.getAppliedOutput());
    Telemetry.setValue("Arm/stage1/statorCurrent", armMotor.getOutputCurrent());
    Telemetry.setValue("Arm/stage1/actualPosition", Math.toDegrees(armEncoder.getAbsolutePosition()));
  }
}
