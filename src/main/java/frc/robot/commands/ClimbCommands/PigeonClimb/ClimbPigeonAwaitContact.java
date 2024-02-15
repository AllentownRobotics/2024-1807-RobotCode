// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands.PigeonClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbPigeonAwaitContact extends Command {
  // Stage 1 of the 3 stage level climber
  Climb climb;
  /** Creates a new ClimbPigeonAwaitContact. */
  public ClimbPigeonAwaitContact(Climb climb) {
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
    
    // uses the tilt of the robot to see
    // which motor contacted first
    if (climb.getTilt() < -ClimbConstants.climbMinTiltedDegrees) {

      climb.setContactOrder(climb.getRightMotor(), climb.getLeftMotor());
      
      // stops the first contacted motor and moves on
      climb.getFirstContact().set(0);
      return true;

    } else if (climb.getTilt() > ClimbConstants.climbMinTiltedDegrees) {

      climb.setContactOrder(climb.getLeftMotor(), climb.getRightMotor());

      // stops the first contacted motor and moves on
      climb.setLeftMotor(0);
      return true;

    } else {

      return false;
    }
  }
}
