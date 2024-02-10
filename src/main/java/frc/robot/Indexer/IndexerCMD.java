// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;

public class IndexerCMD extends Command {
  Indexer indexer;
  
  /** Creates a new IndexerCMD. */
  public IndexerCMD(Indexer indexer) {
  
    this.indexer = indexer;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    indexer.Index(IndexerConstants.indexSpeed);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  
    indexer.Index(0);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
