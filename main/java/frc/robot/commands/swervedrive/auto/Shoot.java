// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Shoot extends Command {
  /** Creates a new testAuton. */
  Timer time;
  Boolean flag;
  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mShooterSub);
    addRequirements(RobotContainer.mIntakeSub);
    time = new Timer();
    flag = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.stop();
    time.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time.start();
    RobotContainer.mShooterSub.Shoot();
    if(time.get()>2.5){
      // flag = true;
      RobotContainer.mIntakeSub.intake();
    }
    else if(time.get() > 2.5 && time.get() <= 3.2){
      RobotContainer.mShooterSub.shooterStop();
      RobotContainer.mIntakeSub.intakeStop();
      flag = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mShooterSub.shooterStop();
    RobotContainer.mIntakeSub.intakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flag;  //was false
  }
}
