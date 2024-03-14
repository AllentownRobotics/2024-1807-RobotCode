// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CMDStoTest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class RevAimEndFireCMD extends Command {
  private Shooter shooterSubsystem;
  private Vision visionSubsystem;
  /** Creates a new RevAimEndFireCMD. */
  public RevAimEndFireCMD(Shooter shooterSubsystem, Vision visionSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.visionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setFlywheelsRPM(ShooterConstants.shootingRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = visionSubsystem.getDistanceToSpeaker();
    if(distance != 0.0)
    {
      shooterSubsystem.setPivotAngle(shooterSubsystem.getAimingAngle(distance));
    }

    visionSubsystem.setShooterReady(shooterSubsystem.atDesiredAngle()&&shooterSubsystem.atDesiredRPM());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setAMPFeeder(ShooterConstants.feederAMPSpeed);
    visionSubsystem.setShooterReady(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
