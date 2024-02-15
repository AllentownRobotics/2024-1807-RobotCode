// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbPigeonRise extends Command {
    // Stage 1 of the 3 stage level climber
  Climb climb;
  /** Creates a new ClimbPigeonRise. */
  public ClimbPigeonRise(Climb climb) {
    this.climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb.setSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // checks if the robot is relatively balanced to ascend faster
    if (Math.abs(climb.getTilt()) < ClimbConstants.climbBalancedDegrees) {

      climb.setLeftMotor(ClimbConstants.climbSpeedFactor);
      climb.setRightMotor(ClimbConstants.climbSpeedFactor);

    // if it isn't balanced, slows down one motor
    // in order to reach a more balanced state
    } else if (climb.getTilt() > 0) {

      climb.setLeftMotor(Math.abs(climb.getSpeed()));
      climb.setRightMotor(Math.abs(climb.getSpeed())/2);

    } else if (climb.getTilt() < 0) {

      climb.setRightMotor(Math.abs(climb.getSpeed()));
      climb.setLeftMotor(Math.abs(climb.getSpeed())/2);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
