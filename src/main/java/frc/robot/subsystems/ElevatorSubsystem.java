// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConfigs;

public class ElevatorSubsystem extends SubsystemBase {
  // motor (neo vortex according to co-engineering pres?)
  private SparkFlex m_elevatorMotor = new SparkFlex(1, MotorType.kBrushless);
  // pid
  private SparkClosedLoopController m_PID;

  private RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getEncoder();

  //btw the positions are fixed 

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorMotor.configure(MotorConfigs.Elevator.elevatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_PID = m_elevatorMotor.getClosedLoopController();
  }

  public void setBrakeMode(boolean brake) {
    m_elevatorMotor.configure((brake ? MotorConfigs.Elevator.elevatorConfig : MotorConfigs.Elevator.coastElevatorConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }


  public void setPower(double power) {
    m_elevatorMotor.set(power);
  }

  public void setPosition(double position) {
    m_PID.setReference(position, ControlType.kPosition);
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
