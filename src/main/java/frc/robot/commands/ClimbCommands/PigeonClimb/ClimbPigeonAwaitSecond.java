// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands.PigeonClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbPigeonAwaitSecond extends Command {
    // Stage 2 of the 3 stage level climber
  Climb climb;
  /** Creates a new ClimbPigeonAwaitSecond. */
  public ClimbPigeonAwaitSecond(Climb climb) {
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
    
    // waits for the second motor to balance the robot and moves on
    if (Math.abs(climb.getTilt()) < ClimbConstants.climbBalancedDegrees) {

      return true;

    } else {

      return false;
    }
  }
}
