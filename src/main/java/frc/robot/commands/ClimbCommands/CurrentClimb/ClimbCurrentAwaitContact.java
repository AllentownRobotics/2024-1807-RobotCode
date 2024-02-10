// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands.CurrentClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbCurrentAwaitContact extends Command {
  Climb climb;
  double current;
  /** Creates a new ClimbCurrentAwaitContact. */
  public ClimbCurrentAwaitContact(Climb climb) {
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gets the baseline current
    climb.setCurrentSnapshot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // checks for a current spike in either motor
    // afterward, it moves to the next step
    if (climb.getCurrentLeft() - climb.getCurrentSnapshot() > ClimbConstants.climbCurrentDifference) {

      // disables the contacted motor and
      // sets it as first contacted
      climb.setLeftMotor(0);
      climb.setContactOrder(climb.getLeftMotor(), climb.getRightMotor());

      return true;
    } else if (climb.getCurrentRight() - climb.getCurrentSnapshot() > ClimbConstants.climbCurrentDifference) {

      // disables the contacted motor and
      // sets it as first contacted
      climb.setRightMotor(0);
      climb.setContactOrder(climb.getRightMotor(), climb.getLeftMotor());

      return true;
    } else {
      return false;
    }
  }
}
