// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ZeroClimbCMD extends Command {
  Climb climbSubsystem;
  boolean leftZeroed = false;
  boolean rightZeroed = false;
  /** Creates a new ZeroClimbCMD. */
  public ZeroClimbCMD(Climb climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climbSubsystem.setLeft(ClimbConstants.climbZeroSpeed);
    climbSubsystem.setRight(ClimbConstants.climbZeroSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climbSubsystem.getLeftLimit())
    {
      climbSubsystem.zeroLeftEncoder();
      leftZeroed = true;
    }

    if(climbSubsystem.getRightLimit())
    {
      climbSubsystem.zeroRightEncoder();
      rightZeroed = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftZeroed && rightZeroed;
  }
}
