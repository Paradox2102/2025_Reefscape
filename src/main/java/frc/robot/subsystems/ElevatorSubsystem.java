// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  // motor (neo vortex according to co-engineering pres?)
  private SparkFlex m_elevatorMotor = new SparkFlex(1, MotorType.kBrushless);
  // pid
  private static double k_p = 0;
  private static double k_i = 0;
  private static double k_d = 0;
  private static double k_f = 0;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  private RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getEncoder();

  //btw the positions are fixed 

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
  }


  public void setPower(double power) {
    m_elevatorMotor.set(power);
  }

  public void setPosition(double position) {
  }

  public double getPosition() {
    return m_elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    //Shows data on SmartDashBoard
    SmartDashboard.putNumber("Elevator Raw Position", getPosition());
    // This method will be called once per scheduler run

  }
}
