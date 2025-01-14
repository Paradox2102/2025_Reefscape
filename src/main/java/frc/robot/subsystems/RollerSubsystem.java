// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_rollerMotor = new SparkFlex(0, MotorType.kBrushless);
  // pid
  private static double k_p = 0;
  private static double k_i = 0;
  private static double k_d = 0;
  private static double k_f = 0;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  private double m_power = 0;

  /** Creates a new RollerSubsystem. */
  public RollerSubsystem() {
  }

  public void setBrakeMode(boolean brake) {
  }

  public void setPower(double power) {
    m_rollerMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
