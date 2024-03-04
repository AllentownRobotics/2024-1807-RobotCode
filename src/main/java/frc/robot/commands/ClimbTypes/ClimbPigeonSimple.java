// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbTypes;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.ClimbCommands.ClimbStart;
import frc.robot.commands.ClimbCommands.ClimbStop;
import frc.robot.subsystems.Climb;

public class ClimbPigeonSimple extends Command {
  Climb climb;
  /** Creates a new ClimbPigeonSimple. */
  public ClimbPigeonSimple(Climb climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new ClimbStart(climb);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if it's too tilted, slows the disadvantaged motor 
    // (only makes one adjustment)
    if (Math.abs(climb.getTilt()) < ClimbConstants.climbTiltMin) {

      climb.setLeftMotor(ClimbConstants.climbSpeedFactor);
      climb.setRightMotor(ClimbConstants.climbSpeedFactor);

    } else if (climb.getTilt() > ClimbConstants.climbTiltMax) {

      climb.setLeftMotor(0);

    } else if (climb.getTilt() < -ClimbConstants.climbTiltMax) {
      
      climb.setLeftMotor(0);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new ClimbStop(climb);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
