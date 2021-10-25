package frc.robot.commands.auto;

import java.util.Map;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class GalacticSearch extends SequentialCommandGroup {
   public GalacticSearch(DriveSubsystem m_robotDrive) {
      TrajectoryConfig config = new TrajectoryConfig(2, 3);
      Trajectory trajectory1R = m_robotDrive.generateTrajectory("GalacticSearch1R", config);        
      Trajectory trajectory1B = m_robotDrive.generateTrajectory("GalacticSearch1B", config);        
      Trajectory trajectory2R = m_robotDrive.generateTrajectory("GalacticSearch2R", config);        
      Trajectory trajectory2B = m_robotDrive.generateTrajectory("GalacticSearch2B", config);        
      // Trajectory trajectory1R = m_robotDrive.loadTrajectoryFromFile("GalacticSearch1R");
      // Trajectory trajectory1B = m_robotDrive.loadTrajectoryFromFile("GalacticSearch1B");
      // Trajectory trajectory2R = m_robotDrive.loadTrajectoryFromFile("GalacticSearch2R");
      // Trajectory trajectory2B = m_robotDrive.loadTrajectoryFromFile("GalacticSearch2B");

      // Set up camera & get PhotonVision result
      String selectedPath = "";
      PhotonCamera m_photon = new PhotonCamera("HD3000");
      m_photon.takeOutputSnapshot();
      PhotonPipelineResult result = m_photon.getLatestResult();
      int targets = 0;

      // Make sure there are targets (if not something is really wrong)
      if (result.hasTargets()) {
         // Check if there is 2 (Path A) or 3 (Path B) balls in the picture
         targets = result.getTargets().size();
         if (targets != 3) {
            // Check if the closest ball is really close (Red) or not (Blue)
            if (result.getBestTarget().getArea() > AutoConstants.kBallArea) {
               // Run path 1R
               selectedPath = "1R";
            }
            else {
               // Run path 1B
               selectedPath = "1B";
            }
         }
         else {
            // Check if the closest ball is really close (Red) or not (Blue)
            if (result.getBestTarget().getArea() > AutoConstants.kBallArea) {
               // Run path 2R
               selectedPath = "2R";
            }
            else {
               // Run path 2B
               selectedPath = "2B";
            }
         }
      }

      SmartDashboard.putString("GalacticSearch", selectedPath);
      SmartDashboard.putNumber("Targets", targets);
      final String path = selectedPath;
      Supplier<Object> i  = ()-> path;
      
      addCommands(
         // Select Command based on selectedPath from above
         new SelectCommand(Map.ofEntries(
            Map.entry("1R", 
               // Reset robot pose to the beginning of 1R and Run it
               m_robotDrive.createCommandForTrajectory(trajectory1R, true)
                           .beforeStarting(() -> m_robotDrive.resetOdometry(trajectory1R.getInitialPose()))
                           .withTimeout(50).withName("GalacticSearch1R")),
            Map.entry("1B", 
               // Reset robot pose to the beginning of 1B and Run it
               m_robotDrive.createCommandForTrajectory(trajectory1B, true)
                           .beforeStarting(() -> m_robotDrive.resetOdometry(trajectory1B.getInitialPose()))
                           .withTimeout(50).withName("GalacticSearch1B")),
            Map.entry("2R", 
               // Reset robot pose to the beginning of 2R and Run it
               m_robotDrive.createCommandForTrajectory(trajectory2R, true)
                           .beforeStarting(() -> m_robotDrive.resetOdometry(trajectory2R.getInitialPose()))
                           .withTimeout(50).withName("GalacticSearch2R")),
            Map.entry("2B", 
               // Reset robot pose to the beginning of 2B and Run it
               m_robotDrive.createCommandForTrajectory(trajectory2B, true)
                           .beforeStarting(() -> m_robotDrive.resetOdometry(trajectory2B.getInitialPose()))
                           .withTimeout(50).withName("GalacticSearch2B"))),
         i)
      );
   }
}