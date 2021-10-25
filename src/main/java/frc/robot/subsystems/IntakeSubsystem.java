// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax frontMotor = new CANSparkMax(IntakeConstants.kFrontIntakePort, MotorType.kBrushless);
  CANSparkMax backMotor = new CANSparkMax(IntakeConstants.kBackIntakePort, MotorType.kBrushless);
  private final WPI_VictorSPX m_dropMotor = new WPI_VictorSPX(IntakeConstants.kDropMotorPort);

  DigitalInput m_lowDropSwitch = new DigitalInput(IntakeConstants.kLowDropSwitchPort);
  DigitalInput m_highDropSwitch = new DigitalInput(IntakeConstants.kHighDropSwitchPort);


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Stops drive motors
    stop();
    
    // Restores default CANSparkMax settings
    frontMotor.restoreFactoryDefaults();
    backMotor.restoreFactoryDefaults();
    
    // Set Idle mode for CANSparkMax (brake)
    frontMotor.setIdleMode(IdleMode.kCoast);
    backMotor.setIdleMode(IdleMode.kCoast);
    
    // Set Smart Current Limit for CAN SparkMax
    frontMotor.setSmartCurrentLimit(30, 40);
    backMotor.setSmartCurrentLimit(30, 40);
  
    // Burn settings into Spark MAX flash
    frontMotor.burnFlash();
    backMotor.burnFlash();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    frontMotor.stopMotor();
    backMotor.stopMotor();
  }
  public void start() {
    frontMotor.set(-1);
    backMotor.set(-0.9); 
  }

  public void reverse() {
    frontMotor.set(0.6);
    backMotor.set(0.7);
  }

  public void drop() {
    m_dropMotor.set(ControlMode.PercentOutput,0.5);
  }

  public void pickup() {
    m_dropMotor.set(ControlMode.PercentOutput,-0.5);
  }
  public void stopDeploy() {
    m_dropMotor.set(ControlMode.PercentOutput,0);
  }

  public void manualDeploy(double intake_spd) {
    m_dropMotor.set(ControlMode.PercentOutput,intake_spd); 
  }

  public boolean getLowDropSwitch(){
    return m_lowDropSwitch.get();
  }
  
  public boolean getHighDropSwitch(){
    return m_highDropSwitch.get();
  }
}
