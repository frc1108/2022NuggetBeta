// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodPositionZero extends CommandBase {
  private final HoodSubsystem m_hood;
  /** Creates a new HoodPositionZero. */
  public HoodPositionZero(HoodSubsystem subsystem) {
    m_hood = subsystem;
    addRequirements(m_hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hood.down();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted) {
      m_hood.reset();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_hood.getLowSwitch();
  }
}
