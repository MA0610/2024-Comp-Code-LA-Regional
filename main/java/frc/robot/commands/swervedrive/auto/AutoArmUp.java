// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.speakerArmPosCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoArmUp extends SequentialCommandGroup {
  /** Creates a new armUpAndShootCommand. */
  public AutoArmUp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
    addCommands(
      new RunCommand(()-> RobotContainer.mArmSub.moveDistanceUp(219, 0.75)).andThen(() ->  new Shoot())  //This makes the path not run and the arm move barely (not in any specefied direction).
    );                                                                                                               //This is becuase there is no is finished but doesn't explain why it doesn't move all the way
  }
  
}

