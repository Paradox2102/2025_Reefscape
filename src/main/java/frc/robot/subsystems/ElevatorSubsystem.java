// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj.DigitalInput;
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
  private DigitalInput m_switch = new DigitalInput(1);

  private ElevatorPosition m_position = ElevatorPosition.L1;

  private static final double k_deadzoneInches = .2;

  private double m_targetPos = m_position.heightInches();

  public Trigger atPosition = new Trigger(
    () -> getPosition() - m_position.heightInches() < k_deadzoneInches);

  public Trigger bottomLimit = new Trigger(
    () -> m_switch.get());


  public enum ElevatorPosition {
    L4(70, "Level 4"),
    L3(45, "Level 3"),
    L2(31, "Level 2"),
    L1(19, "Level 1"),
    RESET(0, "Reset"),
    ALGAE(0, "Algae");

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

  public Trigger isL1 = new Trigger (
    () -> m_position == ElevatorPosition.L1);

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

  public Command setTargetPos(ElevatorPosition pos) {
    return Commands.runOnce(() -> {setPosition(pos);}, this);
  }

  public Command goToPosition() {
    return Commands.run(() -> {
      m_PID.setReference(m_position.heightInches(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }, this).until(atPosition);
  }

  public Command resetPosition() {
    return Commands.run(() -> {
      m_PID.setReference(.5, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }, this).until(() -> getPosition() < k_deadzoneInches);
  }

  public Command manualMove(DoubleSupplier up) {
    return Commands.run(() -> {
      double direction = -up.getAsDouble();
      m_elevatorMotor.set(direction > 0 ? .2 : -.2);
    }, this);
  }


  @Override
  public void periodic() {
    //Shows data on SmartDashBoard
    double pos = getPosition();
    m_targetPos = m_position.heightInches();
    double output = m_elevatorMotor.getAppliedOutput();
    SmartDashboard.putNumber("Elevator Position", pos);
    SmartDashboard.putNumber("elevator speed", m_elevatorEncoder.getVelocity());
    SmartDashboard.putNumber("elevator output", output);
    SmartDashboard.putString("Elevator Target", m_position.getName());
    if (!bottomLimit.getAsBoolean()){
      m_elevatorEncoder.setPosition(0);
    } else if (pos > 73){
      setPower(0);
      m_elevatorEncoder.setPosition(72.5); //if encoder is reading out number 0.5 inch above max travel, set height to max travel
    }
    // This method will be called once per scheduler run
  } 
}
