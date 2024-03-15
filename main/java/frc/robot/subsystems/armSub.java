// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.armCommand;

public class armSub extends SubsystemBase {
  /** Creates a new armSub. */
  CANSparkMax armL = new CANSparkMax(Constants.armMotorL,MotorType.kBrushless);
  CANSparkMax armR = new CANSparkMax(Constants.armMotorR, MotorType.kBrushless);
  DigitalInput input = new DigitalInput(0);
  DutyCycleEncoder encoder = new DutyCycleEncoder(input);
  

  public armSub() {

    armL.setInverted(true);
    armR.setInverted(false);
    armL.setIdleMode(IdleMode.kBrake);
    armR.setIdleMode(IdleMode.kBrake);

    
  
  }

  public void resetEncoder(){
    encoder.reset();
  }

  public double getRot(){
    return encoder.get();
  }

  //Different for both Forward and Backward
  //less than for Negative and greater than for Positive
  public void moveDistanceUp(double value,double speed){
    double ASpeed1 = Math.abs(speed);
    double AValue1 = Math.abs(value);

    // if(encoder.get()*100<AValue1){
    armR.set(ASpeed1);
    armL.set(ASpeed1);
      if(encoder.get()*1000>=AValue1){
        armR.set(0);
        armL.set(0);
      }
  // }
  }

  public void moveDistanceDown(double value,double speed){
    double ASpeed2 = -Math.abs(speed);
    double AValue2 = Math.abs(value);
      // if(encoder.get()*100>AValue2){
        armR.set(ASpeed2);
        armL.set(ASpeed2);

        if(encoder.get()*1000<=AValue2){
          armR.set(0);
          armL.set(0);
        }
      // }

  }

  public void setPower(double power){
    if (power >= 0.60){
      armL.set(0.60);
      armR.set(0.60);
    }
    else if (power <= -0.60){
      armL.set(-0.60);
      armR.set(-0.60);
    }
    else{
    armL.set(power);
    armR.set(power);
    }
  }

  public void armUp(){
    armL.set(0.50);
    armR.set(0.50);
  }

  public void armDown(){
  armL.set(-0.50);
  armR.set(-0.50);
  }

  public void armStop(){
  armL.set(0.00);  //0.004
  armR.set(0.00);  //0.004
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //RobotContainer.mArmSub.setDefaultCommand(new armCommand());
  }
}
