// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hood;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.HoodSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoodGoToAngle extends ProfiledPIDCommand {
  /** Creates a new HoodToAngle. */

  HoodSubsystem m_hood;

  public HoodGoToAngle(double targetAngleDegrees, HoodSubsystem hood) {
    super(
      // The ProfiledPIDController used by the command
      new ProfiledPIDController(
          // The PID gains
          HoodConstants.kHoodP,
          HoodConstants.kHoodI,
          HoodConstants.kHoodD,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(0, 0)),
      // This should return the measurement
      hood::getAngle,
      // This should return the goal (can also be a constant)
      targetAngleDegrees,
      // This uses the output
      (output, setpoint) -> {
        hood.set(output);
      },
      hood);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    // Configure for angle input
    getController().enableContinuousInput(-180, 180);

    getController().setTolerance(HoodConstants.kTurnToleranceDeg, HoodConstants.kTurnRateToleranceDegPerS);

    m_hood = hood;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }
}
