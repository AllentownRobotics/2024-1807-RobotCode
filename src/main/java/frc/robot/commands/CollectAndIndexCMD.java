// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.Constants.CollectorConstants;
import frc.robot.Utils.Constants.IndexerConstants;
import frc.robot.Utils.Constants.ShooterConstants;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class CollectAndIndexCMD extends Command {
  Collector collectorSubsystem;
  Indexer indexerSubsystem;
  Shooter shootersubsystem;
  /** Creates a new CollectAndIndexCMD. */
  public CollectAndIndexCMD(Collector collectorSubsystem, Indexer indexerSubsystem, Shooter shooterSubsystem) {
    this.collectorSubsystem = collectorSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shootersubsystem = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collectorSubsystem,indexerSubsystem,shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectorSubsystem.collect(CollectorConstants.collectorSpeed);
    indexerSubsystem.index(IndexerConstants.indexerSpeed);
    shootersubsystem.setAMPFeeder(ShooterConstants.feederAMPSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    collectorSubsystem.collect(0);
    indexerSubsystem.index(0);
    shootersubsystem.setAMPFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
