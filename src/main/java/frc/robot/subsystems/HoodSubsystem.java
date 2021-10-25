// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;

public class HoodSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_motor = new WPI_VictorSPX(HoodConstants.kHoodMotorPort);
  private final Encoder m_encoder = new Encoder(HoodConstants.kHoodEncoderPortA,HoodConstants.kHoodEncoderPortB); 
  private final DigitalInput m_lowSwitch = new DigitalInput(HoodConstants.kLowSwitchPort);

  public HoodSubsystem() {
    m_motor.configFactoryDefault();
    m_motor.setNeutralMode(NeutralMode.Brake);

    this.setDefaultCommand(new RunCommand(() -> stop(), this).withName("Stop"));
  }

  /**
  * Set speed of hood motor
  * @param speed positive speed 0-1 hood rotates upward
  */
  public void set(double speed) {
    // Set speed to zero when a low switch or encoder at soft limit of 150
    // speed = ((speed < 0 && getLowSwitch()) || (speed > 0 && getEncoderValue() > 150))?0:speed;
    m_motor.set(-speed);  // invert speed so negative input is down 
  }

  public void down(){
    set(-0.5);
  }

  public void up(){
    set(0.7);
  }

  /**
  * Stop the subsystem motors
  */
  public void stop() {
    m_motor.stopMotor();
  }

  public void reset() {
    m_encoder.reset();
  }

   public double getEncoderValue() {
    return -m_encoder.getDistance();
  }
  
   public double getAngle() {
    return HoodConstants.kHoodStartingAngle
           -(getEncoderValue()*HoodConstants.kHoodDegreesPerCount);
  }

  public boolean getLowSwitch(){
   return !m_lowSwitch.get();  // logic inverted
  }
}
