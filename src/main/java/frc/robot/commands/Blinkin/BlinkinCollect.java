// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Blinkin;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BlinkinConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Collector;

public class BlinkinCollect extends Command {
   Blinkin blinkin;
   Collector collect;
   
  /** Creates a new BlinkinCollect. */
  public BlinkinCollect(Blinkin blinkin, Collector collect) {
    this.blinkin = blinkin;
    this.collect = collect;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (collect.CollectingLight() == true) {
    blinkin.BlinkinSet(BlinkinConstants.blinkinOrange);
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
