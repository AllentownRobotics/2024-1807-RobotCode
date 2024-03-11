// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCMDs;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterCMDs.LowLevelCMDs.RunAMPMotorsCMD;
import frc.robot.commands.ShooterCMDs.LowLevelCMDs.SetPivotAngleCMD;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreAMPCMD extends SequentialCommandGroup {
  /** Creates a new ScoreAMPCommand. */
  public ScoreAMPCMD(Shooter shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetPivotAngleCMD(ShooterConstants.AMPAngle, shooterSubsystem),
      Commands.waitUntil(shooterSubsystem::atDesiredAngle),
      new RunAMPMotorsCMD(shooterSubsystem));
  }
}
