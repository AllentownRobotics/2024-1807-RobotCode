// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CollectorConstants;
import frc.robot.commands.Collector.CollectorCMD;
import frc.robot.commands.Indexer.IndexerCMD;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Indexer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CollectAndIndex extends SequentialCommandGroup {
  Collector collector;
  Indexer indexer;
  
  /** Creates a new CollectAndIndex. */
  public CollectAndIndex(Collector collector, Indexer indexer) {
    
    this.collector = collector;
    
    this.indexer = indexer;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CollectorCMD(collector),
      Commands.waitSeconds(CollectorConstants.collectSeconds),
      new IndexerCMD(indexer)
    );
  }
}
