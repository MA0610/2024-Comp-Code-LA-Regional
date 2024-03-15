// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intakeSub extends SubsystemBase {
  /** Creates a new intakeSub. */
  TalonFX intake;
  public intakeSub() {
  intake = new TalonFX(Constants.intakeMotor);

  intake.setNeutralMode(NeutralModeValue.Brake);
  intake.setInverted(true);
  }

  public void intake(){
    intake.set(0.20);
  }

  public void intakeBack(){
    intake.set(-0.10);
  }

  public void intakeStop(){
    intake.set(0);
  }

  public void ampIntakeSpeed(){
    intake.set(0.10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
