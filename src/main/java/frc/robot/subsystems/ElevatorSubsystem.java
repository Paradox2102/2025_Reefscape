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
  private DigitalInput m_switch = new DigitalInput(0);

  private ElevatorPosition m_position = ElevatorPosition.L4;
  private ElevatorPosition m_algaePosition = ElevatorPosition.ALGAE_LOW;

  private static final double k_deadzoneInches = 0.5;

  private double m_targetPos = m_position.heightInches();

  public Trigger atPosition = new Trigger(
    () -> Math.abs(getPosition() - m_position.heightInches()) < k_deadzoneInches);

  public Trigger bottomLimit = new Trigger(
    () -> m_switch.get());


  public enum ElevatorPosition {
    L4(68.2, "Level 4"), // 70.2
    L3(43, "Level 3"),
    L2(31.5, "Level 2"),
    L1(18, "Level 1"),
    RESET(0, "Reset"),
    ALGAE_HIGH(33.48, "Algae High"),
    ALGAE_LOW(18, "Algae Low");

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
    m_elevatorMotor.configure(MotorConfigs.Elevator.leaderConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_followerMotor.configure(MotorConfigs.Elevator.followerConfig, ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
    m_PID = m_elevatorMotor.getClosedLoopController();
    
  }

  public Trigger isLow = new Trigger (
    () -> m_position == ElevatorPosition.L1 || m_position == ElevatorPosition.L2);

  private boolean m_manual = false;

  public Trigger manual = new Trigger(
    () -> m_manual
  );

  private void setPosition(ElevatorPosition position) {
    m_position = position;
  }

  public void setBrakeMode(boolean brake) {
    m_elevatorMotor.configure((brake ? MotorConfigs.Elevator.leaderConfig : MotorConfigs.Elevator.coastElevatorConfig), ResetMode.kResetSafeParameters,
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
    return Commands.runOnce(() -> {setPosition(pos);});
  }

  public Command setAlgaePosition(ElevatorPosition pos) {
    return Commands.runOnce(() -> {m_algaePosition = pos;});
  }

  public Command goToPosition() {
    return Commands.run(() -> {
      m_manual = false;
      m_PID.setReference(m_position.heightInches(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }, this);//.until(atPosition.debounce(.1));
  }

  public Command goToPosition(boolean manual) {
    return Commands.run(() -> {
      m_PID.setReference(m_position.heightInches(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      m_manual = manual;
    }, this);//.until(atPosition.debounce(.1));
  }

  public Command goToAlgaePosition() {
    return Commands.run(() -> {
      m_manual = false;
      m_PID.setReference(m_algaePosition.heightInches(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }, this);
  }

  public Command resetPosition() {
    return Commands.run(() -> {
      m_manual = false;
      m_PID.setReference(0, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }, this).until(() -> getPosition() < 0.25);
  }

  public Command manualMove(DoubleSupplier up) {
    return Commands.run(() -> {
      m_manual = true;
      double direction = -up.getAsDouble();
      m_elevatorMotor.set(direction > 0 ? .1 : -.1);
    }, this).finallyDo(() -> {
      m_elevatorMotor.set(.02);
    });
  }

  public Command setManual(boolean manual) {
    return Commands.runOnce(() -> {
      m_manual = manual;
    });
  }

  public Command resetReading() {
    return Commands.runOnce(() -> {
      m_elevatorEncoder.setPosition(0);
    });
  }


  @Override
  public void periodic() {
    //Shows data on SmartDashBoard
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putString("Elevator Target", m_position.getName());
    if (!bottomLimit.getAsBoolean()){
      m_elevatorEncoder.setPosition(0);
    }
    // This method will be called once per scheduler run
  } 
}
