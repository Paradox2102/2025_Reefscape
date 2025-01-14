// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  // motor
  private SparkFlex m_pivotMotor = new SparkFlex(0, MotorType.kBrushless);
  // pid
  private static double k_p = 0;
  private static double k_i = 0;
  private static double k_d = 0;
  private static double k_f = 0;
  private PIDController m_PID = new PIDController(k_p, k_i, k_d);

  // private static double k_deadzone = 0;

  //set relative w/ absolute -> depend on relative

  private RelativeEncoder m_rEncoder = m_pivotMotor.getEncoder();
  private double m_power;
  private double m_setPosition;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    // setBrakeMode(true);
  }

  public void setBrakeMode(boolean brake) {
    // m_pivotMotor.setIdleMode(brake? IdleMode.kBrake : IdleMode.kCoast);
  }

  public void setPower(double power) {
    m_pivotMotor.set(power);
  }

  public double getAngleInDegrees() {
    return m_rEncoder.getPosition();
  }

  public void setPositonInDegrees(double degrees) {
    m_setPosition = degrees;
  }

  @Override
  public void periodic() {
    //Show the data on SmartDashBoard
    SmartDashboard.putNumber("Pivot Raw Position", getAngleInDegrees());

    // This method will be called once per scheduler run
    double FF;
    double angle = getAngleInDegrees();

    FF = k_f * Math.sin(Math.toRadians(angle));
  }
}
