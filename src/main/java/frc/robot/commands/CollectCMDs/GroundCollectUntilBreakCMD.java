// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CollectCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CollectorConstants;
import frc.robot.subsystems.Collector;

public class GroundCollectUntilBreakCMD extends Command {
  Collector collectorSubsystem;
  /** Creates a new GroundCollectUntilBreakCMD. */
  public GroundCollectUntilBreakCMD(Collector collectorSubsystem) {
    this.collectorSubsystem = collectorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collectorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    collectorSubsystem.collect(CollectorConstants.collectorSpeed);
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
    return collectorSubsystem.getCollectorBeamBreak();
  }
}
