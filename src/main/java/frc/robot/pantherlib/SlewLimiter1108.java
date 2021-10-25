// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pantherlib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;


/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class SlewLimiter1108 {
  private final double m_rateLimitA;
  private final double m_rateLimitD;
  private double m_prevVal;
  private double m_prevTime;

  /**
   * Creates a new SlewRateLimiter with the given rate limit and initial value.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   * @param initialValue The initial value of the input.
   */
  public SlewLimiter1108(double rateLimitA, double rateLimitD, double initialValue) {
    m_rateLimitA = rateLimitA;
    m_rateLimitD = rateLimitD;
    m_prevVal = initialValue;
    m_prevTime = Timer.getFPGATimestamp();
  }

  /**
   * Creates a new SlewRateLimiter with the given rate limit and an initial value of zero.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public SlewLimiter1108(double rateLimitA, double rateLimitD) {
    this(rateLimitA, rateLimitD, 0);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = Timer.getFPGATimestamp();
    double elapsedTime = currentTime - m_prevTime;
    if (input - m_prevVal < 0) {
      m_prevVal +=
        MathUtil.clamp(input - m_prevVal, -m_rateLimitA * elapsedTime, m_rateLimitA * elapsedTime);
    } else {
       m_prevVal +=
       MathUtil.clamp(input - m_prevVal, -m_rateLimitD * elapsedTime, m_rateLimitD * elapsedTime);
    } 
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = Timer.getFPGATimestamp();
  }
}

