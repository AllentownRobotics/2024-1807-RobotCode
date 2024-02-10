// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands.CurrentClimb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbCurrentRise extends Command {
  Climb climb;
  /** Creates a new ClimbCurrentRise. */
  public ClimbCurrentRise(Climb climb) {
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

    // slows down whichever motor has more current (bearing more weight) to balance
    if (climb.getCurrentLeft() > climb.getCurrentRight()) {

      climb.setLeftMotor(ClimbConstants.climbSpeedFactor / 2);
      climb.setRightMotor(ClimbConstants.climbSpeedFactor);

    } else if (climb.getCurrentRight() > climb.getCurrentLeft()) {

      climb.setRightMotor(ClimbConstants.climbSpeedFactor / 2);
      climb.setLeftMotor(ClimbConstants.climbSpeedFactor);

    } else {

      climb.setLeftMotor(ClimbConstants.climbSpeedFactor);
      climb.setRightMotor(ClimbConstants.climbSpeedFactor);

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
