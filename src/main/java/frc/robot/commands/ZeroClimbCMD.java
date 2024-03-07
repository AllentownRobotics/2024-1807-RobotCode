// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ZeroClimbCMD extends Command {
  Climb climbSubsytem;
  /** Creates a new ZeroClimbCMD. */
  public ZeroClimbCMD(Climb climbSubsystem) {
    this.climbSubsytem = climbSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(!climbSubsytem.getLeftLimit() || !climbSubsytem.getRightLimit())
    {
        if(!climbSubsytem.getLeftLimit())
        {
          climbSubsytem.setLeft(.1);
        }
        else
        {
          climbSubsytem.setLeft(.1);
          climbSubsytem.zeroLeftEncoder();
        }

        if(!climbSubsytem.getRightLimit())
        {
          climbSubsytem.setRight(.1);
        }
        else
        {
          climbSubsytem.setRight(0);
          climbSubsytem.zeroRightEncoder();
        }
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
