// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMainPort = 1;
    public static final int kLeftFollowPort = 3;
    public static final int kRightMainPort = 2;
    public static final int kRightFollowPort = 4;

    public static final double kTrackwidthMeters = Units.inchesToMeters(20);
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 42;
    public static final double kGearRatio = 10.71;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
    public static final double kEncoderDistanceConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(kGearRatio));
    public static final double kEncoderVelocityConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(60*kGearRatio));

    public static final double ksVolts = 0.198;   //0.169
    public static final double kvVoltSecondsPerMeter = 2.86;  //2.24
    public static final double kaVoltSecondsSquaredPerMeter = 0.365;  //0.0435
    public static final double kPDriveVel = 2;  //2.4 8/14 2.24 Tuning to get better PIDF response

    public static final Boolean kGyroReversed = false;

    public static final double kTurnP = 0.0475; //0.94, 0.125
    public static final double kTurnI = 0.00;
    public static final double kTurnD = 0.00175; //0.04, 0.0085
    public static final double kMinCommand = 0.0;

    public static final double kMaxTurnRateDegPerS = 360;
    public static final double kMaxTurnAccelerationDegPerSSquared = 480;

    public static final double kTurnToleranceDeg = 4; //0.5
    public static final double kTurnRateToleranceDegPerS = 8;

    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(8);
  }

  public static final class ShooterConstants {
    public static final int kStopperServoPort = 9;

    public static final int kRightMotorPort = 20;
    public static final int kLeftMotorPort = 21;

    public static final double kP = 0.0012;
    public static final double kI = 0.0;
    public static final double kD = 0.00012;
    public static final double kIz = 0.0;
    public static final double kFF = 0.0001875;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;

    public static final double kmaxRPM = 5700;
	  public static final double kIdleRPM = 1500;
  }



  public static final class HoodConstants {
    public static final int kHoodMotorPort = 33; //22

    public static final int kHoodEncoderPortA = 4;
    public static final int kHoodEncoderPortB = 5;
    public static final double kHoodStartingAngle = 66.0; //measured via phone app
    public static final double kHoodDegreesPerCount = ((93-24) / 1520.0); //phone app measurement
    public static int kLowSwitchPort = 2;

	public static double kHoodP = 0.01;
	public static double kHoodI = 0;
	public static double kHoodD = 0;

	public static double kTurnRateToleranceDegPerS;

	public static double kTurnToleranceDeg;
  }
  public static final class IntakeConstants {
    public static final int kFrontIntakePort = 40;
    public static final int kBackIntakePort = 41;
	  public static final int kDropMotorPort = 42;
	  public static int kLowDropSwitchPort = 7;
	  public static int kHighDropSwitchPort = 8;
  }
  public static final class HotDogConstants {
    public static final int kHotDogPort = 30; //30
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(8);
    public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(8);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    
    public static final double kmaxCentripetalAccelerationMetersPerSecondSq = 0.03;
    public static final double  kDifferentialDriveKinematicsConstraint = 0.3;

    public static final double kBallArea = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
	  public static int kOperatorControllerPort = 1;
	public static int kButtonBoxPort = 2;
  }
}
