// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorConfigs;
import frc.robot.Constants.COConstants;

public class CoralOuttakeSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_coralMotor = new SparkFlex(COConstants.k_coralMotor, MotorType.kBrushless);
  // private SparkFlex m_practiceMotor;
  private SparkClosedLoopController m_pid;
  private RelativeEncoder m_encoder;

  private static final double k_ejectedCurrent = 30;

  private static final double k_outtakeSpeed = 2500;
  private static final double k_outtakeDeadband = 100;
  private static final double k_l1Speed = 500;
  private boolean m_isL1 = false;

  // I'm still unsure that testing against a current level is going to be reliable
  // here. Also, consider using distance travelled instead of a time. We can talk
  // about how to do that in a Trigger. -Gavin
  public final Trigger ejectedCoral = new Trigger(
      () -> getCurrentDraw() < k_ejectedCurrent && getSpeedMotorRPM() >= (m_isL1 ? k_l1Speed : k_outtakeSpeed) - k_outtakeDeadband)
      .debounce(.25, DebounceType.kRising);

  /** Creates a new RollerSubsystem. */
  public CoralOuttakeSubsystem() {
    m_coralMotor.configure(MotorConfigs.CoralOuttake.config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_pid = m_coralMotor.getClosedLoopController();
    m_encoder = m_coralMotor.getEncoder();
  }

  public void setBrakeMode(boolean brake) {
    m_coralMotor.configure((brake ? MotorConfigs.CoralOuttake.config : MotorConfigs.CoralOuttake.config),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    m_coralMotor.set(power);
  }

  public double getSpeedMotorRPM() {
    return m_encoder.getVelocity();
  }

  public void setSpeed(double speed) {
    SmartDashboard.putNumber("coral target speed", speed);
    m_pid.setReference(speed, ControlType.kVelocity);
  }

  public double getCurrentDraw() {
    return m_coralMotor.getOutputCurrent();
  }

  public Command runSpeed(double speed) {
    return Commands.run(() -> {
      setSpeed(speed);
    }, this);
  }

  public Command ejectCoral(BooleanSupplier L1) {
    m_isL1 = L1.getAsBoolean();
    System.out.println("yahoo");
    return Commands.run(() -> {
      setSpeed(m_isL1 ? k_l1Speed : k_outtakeSpeed);
    }, this).until(ejectedCoral);
  }

  public Command stop() {
    return Commands.runOnce(() -> {
      setPower(0);
    }, this);
  }

  public Command runOut() {
    return Commands.run(() -> {
      setPower(.25);
    }, this);
  }

  public Command holdCoral() {
    return Commands.run(() -> {
      setPower(-0.05);
    }, this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Current", getCurrentDraw());
    SmartDashboard.putNumber("coral speed rpm", getSpeedMotorRPM());
    // This method will be called once per scheduler run
  }
}