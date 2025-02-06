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
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.MotorConfigs;

public class ElevatorSubsystem extends SubsystemBase {
  // motor (neo vortex according to co-engineering pres?)
  private SparkFlex m_elevatorMotor = new SparkFlex(Constants.ElevatorConstants.k_elevatorMotor, MotorType.kBrushless);
  private SparkFlex m_followerMotor = new SparkFlex(Constants.ElevatorConstants.k_elevatorFollower, MotorType.kBrushless);
  // pid
  private SparkClosedLoopController m_PID;

  private RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getEncoder();

  private ElevatorPosition m_position = ElevatorPosition.L1;

  private static final double k_deadzoneInches = 3;
  private double m_targetPos = 0;

  public Trigger atPosition = new Trigger(
    () -> getPosition() - m_position.heightInches() < k_deadzoneInches);

  public enum ElevatorPosition {
    L4(0, "Level 4"),
    L3(0, "Level 3"),
    L2(0, "Level 2"),
    L1(0, "Level 1");

    private double m_heightInches;
    private String m_name;
    ElevatorPosition(double heightInches, String name) {
      m_heightInches = heightInches;
      m_name = name;
    }

    public double heightInches() {
      return m_heightInches;
    }

    public String getName() {
      return m_name;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_elevatorMotor.configure(MotorConfigs.Elevator.elevatorConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_followerMotor.configure(MotorConfigs.Elevator.followerConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
    m_PID = m_elevatorMotor.getClosedLoopController();
  }

  // FIXME: This should be a command factory. -Gavin
  public void setPosition(ElevatorPosition position) {
    m_position = position;
  }

  public void setBrakeMode(boolean brake) {
    m_elevatorMotor.configure((brake ? MotorConfigs.Elevator.elevatorConfig : MotorConfigs.Elevator.coastElevatorConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }


  public void setPower(double power) {
    m_elevatorMotor.set(power);
  }

  public ElevatorPosition getPreset() {
    return m_position;
  }

  public double getPosition() {
    return m_elevatorEncoder.getPosition();
  }

  public Command goToPosition() {
    return Commands.run(() -> {
      m_targetPos = 15;//m_position.heightInches();
      // m_PID.setReference(15, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }, this);
  }

  public Command resetPosition() {
    return Commands.run(() -> {
      m_targetPos = 1;
      // m_PID.setReference(2, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }, this);
  }

  public Command manualMove(boolean up) {
    return Commands.run(() -> {
      m_elevatorMotor.set(up ? .2 : -.2);
    }, this);
  }


  @Override
  public void periodic() {
    //Shows data on SmartDashBoard
    double pos = getPosition();
    SmartDashboard.putNumber("Elevator Raw Position", pos);
    if (Math.abs(m_targetPos - pos) > k_deadzoneInches) {
      setPower((.2 * Math.signum(m_targetPos - pos)) + Constants.ElevatorConstants.k_f);
    } else {
      setPower(Constants.ElevatorConstants.k_f);
    }

    // This method will be called once per scheduler run
  } 
}
