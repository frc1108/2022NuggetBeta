package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Slalom extends SequentialCommandGroup {
  public Slalom(DriveSubsystem m_robotDrive) {       
      TrajectoryConfig config = new TrajectoryConfig(2, 3);
      
      Trajectory trajectory1 = m_robotDrive.generateTrajectory("Slalom1", config);
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajectory1.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Slalom1")
      );
  }
}