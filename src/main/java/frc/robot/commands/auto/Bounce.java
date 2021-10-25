package frc.robot.commands.auto;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class Bounce extends SequentialCommandGroup {
  public Bounce(DriveSubsystem m_robotDrive) {  
    TrajectoryConfig fwdConfig = new TrajectoryConfig(2, 3);
    TrajectoryConfig revConfig = new TrajectoryConfig(2, 3).setReversed(true);

    Trajectory trajectory1 = m_robotDrive.generateTrajectory("Bounce1", fwdConfig);
    Trajectory trajectory2 = m_robotDrive.generateTrajectory("Bounce2", revConfig);
    Trajectory trajectory3 = m_robotDrive.generateTrajectory("Bounce3", fwdConfig);
    Trajectory trajectory4 = m_robotDrive.generateTrajectory("Bounce4", revConfig);
      
      addCommands(
          new InstantCommand(() -> {
              m_robotDrive.resetOdometry(trajectory1.getInitialPose());
          }),
          m_robotDrive.createCommandForTrajectory(trajectory1, false).withTimeout(50).withName("Bounce1"),
          m_robotDrive.createCommandForTrajectory(trajectory2, false).withTimeout(50).withName("Bounce2"),
          m_robotDrive.createCommandForTrajectory(trajectory3, false).withTimeout(50).withName("Bounce3"),
          m_robotDrive.createCommandForTrajectory(trajectory4, false).withTimeout(50).withName("Bounce4")
      );
  }
}