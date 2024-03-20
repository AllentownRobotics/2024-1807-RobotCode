// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCMDs.LowLevelCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class SetAngleDistanceCMD extends Command {
  private Shooter shooterSubsystem;
  private Vision visionSubsystem;
  public SetAngleDistanceCMD(Shooter shooterSubsystem, Vision visionSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.visionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setPivotAngle(shooterSubsystem.getAimingAngle(visionSubsystem.getDistanceToSpeaker()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSubsystem.atDesiredAngle();
  }
}
