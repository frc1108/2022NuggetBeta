// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.auto.GalacticSearch;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchParallelGroup extends ParallelCommandGroup {
  /** Creates a new GalacticSearchParallelGroup. */
  public GalacticSearchParallelGroup(DriveSubsystem m_robotDrive, IntakeSubsystem m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
          new InstantCommand(() -> m_intake.start(),m_intake).withTimeout(15),
          new GalacticSearch(m_robotDrive)
    );
  }
}
