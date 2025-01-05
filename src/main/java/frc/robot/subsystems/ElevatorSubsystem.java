// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  // motor (neo vortex according to co-engineering pres?)
  private SparkFlex m_elevatorMotor = new SparkFlex(0, MotorType.kBrushless);
  // pid
  private double k_p = 0;
  private double k_i = 0;
  private double k_d = 0;
  private double k_f = 0;

  //btw the positions are fixed 

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
  }

  public void setBrakeMode(boolean brake) {
  }

  public void setPower(double power) {
    m_elevatorMotor.set(power);
  }

  public void setPosition(double position) {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
