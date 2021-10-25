// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.Constants.HotDogConstants;

public class HotDog extends SubsystemBase {
  /** Creates a new HotDog and adds mustard. */
  private final VictorSPX victor0 = new VictorSPX(HotDogConstants.kHotDogPort);
  public HotDog() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void stop() {
    victor0.set(ControlMode.PercentOutput, 0);
  }
  public void start() {
    victor0.set(ControlMode.PercentOutput, -0.75);
  }
}
