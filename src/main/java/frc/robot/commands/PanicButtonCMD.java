// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class PanicButtonCMD extends Command {
  private Collector collectorSubsystem;
  private Indexer indexerSubsystem;
  private Shooter shooterSubsystem;
  /** Creates a new PanicButtonCMD. */
  public PanicButtonCMD(Collector collectorSubsystem, Indexer indexerSubsystem, Shooter shooterSubsystem) {
    this.collectorSubsystem = collectorSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collectorSubsystem, indexerSubsystem, shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectorSubsystem.collect(-CollectorConstants.collectorSpeed);
    indexerSubsystem.index(-IndexerConstants.indexerSpeed);
    shooterSubsystem.setAMPFeeder(-ShooterConstants.feederAMPSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectorSubsystem.collect(0);
    indexerSubsystem.index(0);
    shooterSubsystem.setAMPFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
