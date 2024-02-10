// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands.CurrentClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbCurrentAwaitSecond extends Command {
  Climb climb;
  /** Creates a new ClimbCurrentAwaitSecond. */
  public ClimbCurrentAwaitSecond(Climb climb) {
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // checks the second motor and waits for a current spike
    if (climb.getSecondContact().getOutputCurrent() - climb.getCurrentSnapshot() > 
    ClimbConstants.climbCurrentDifference) {
      
      // stops the second motor and moves on
      climb.getSecondContact().set(0);

      return true;
    } else {

      return false;
    }
  }
}
