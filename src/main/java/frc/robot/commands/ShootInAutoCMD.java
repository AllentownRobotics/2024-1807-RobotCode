// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.CMDStoTest.RevAimEndCMD;
import frc.robot.commands.DriveCMDs.RotateToSpeakerCMD;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootInAutoCMD extends SequentialCommandGroup {
  /** Creates a new ShootInAutoCMD. */
  public ShootInAutoCMD(Shooter shooterSubsystem, Vision visionSubsystem, DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(new RotateToSpeakerCMD(driveTrain, visionSubsystem), new RevAimEndCMD(shooterSubsystem, visionSubsystem)),
      new InstantCommand(() -> shooterSubsystem.setAMPFeeder(ShooterConstants.feederAMPSpeed)));
  }
}
