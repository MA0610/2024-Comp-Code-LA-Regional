// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class speakerArmPosCommand extends Command {
  /** Creates a new speakerArmPosCommand. */
  public speakerArmPosCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mArmSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.mArmSub.getRot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //RobotContainer.mArmSub.moveDistanceUp((82.1)/100, 0.50);
    RobotContainer.mArmSub.moveDistanceUp(219, 0.75 ); //Was 217
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.mArmSub.armStop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
