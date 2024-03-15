// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class FeedShotFromMidLine extends Command {
  private Shooter shooterSubsystem;
  /** Creates a new ManShootCurrentAngle. */
  public FeedShotFromMidLine(Shooter shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setPivotAngle(ShooterConstants.feedingAngle);
    shooterSubsystem.setFlywheelsRPM(ShooterConstants.feedingMidRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setAMPFeeder(ShooterConstants.feederAMPSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.atDesiredAngle()&&shooterSubsystem.atDesiredRPM();
  }
}
