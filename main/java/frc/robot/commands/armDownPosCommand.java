// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class armDownPosCommand extends Command {
  /** Creates a new armDownPosCommand. */
  public armDownPosCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.mArmSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
    RobotContainer.mArmSub.moveDistanceDown(45, 0.75);
    //RobotContainer.mArmSub.moveDistanceUp(80, 0.3);
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
