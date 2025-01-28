// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.MotorConfigs;
import frc.robot.Constants.MotorConfigs.CoralOuttake;
import frc.robot.Constants.MotorConfigs.Elevator;

public class CoralOuttakeSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_coralLeftMotor = new SparkFlex(34, MotorType.kBrushless);
  private SparkClosedLoopController m_PID;

  private static final double k_ejectedCurrent = 20;
  private static final double k_intakeCurrent = 20;

  private static final double k_intakePower = .5;
  private static final double k_outtakePower = -.5;

  public final Trigger ejectedCoral = new Trigger(
    () -> getCurrentDraw() < k_ejectedCurrent)
    .debounce(.1, DebounceType.kRising);

  public final Trigger hasCoral = new Trigger(
    () -> getCurrentDraw() > k_intakeCurrent)
    .debounce(.1, DebounceType.kRising);

  /** Creates a new RollerSubsystem. */
  public CoralOuttakeSubsystem() {
    m_coralLeftMotor.configure(MotorConfigs.CoralOuttake.leftConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean brake) {
    m_coralLeftMotor.configure((brake ? MotorConfigs.CoralOuttake.leftConfig : MotorConfigs.CoralOuttake.coastLeftConfig), ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  public void configurePID(double F, double P, double I, double D){
    CoralOuttake.p = P;
    CoralOuttake.i = I;
    CoralOuttake.d = D;
    CoralOuttake.f = F;
    m_coralLeftMotor.configure(Elevator.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPower(double power) {
    m_coralLeftMotor.set(power);
  }

  public void test(double speed) {
    m_PID.setReference(speed, ControlType.kVelocity);
  }

  public double getCurrentDraw() {
    return m_coralLeftMotor.getOutputCurrent();
  }

  public Command ejectCoral() {
    return Commands.runOnce(() -> {
      setPower(k_outtakePower);
    }, this).until(ejectedCoral);
  }

  public Command intakeCoral() {
    return Commands.runOnce(() -> {
      setPower(k_intakePower);
    }, this).until(hasCoral);
  }

  public Command stop() {
    return Commands.run(() -> {
      setPower(0);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}