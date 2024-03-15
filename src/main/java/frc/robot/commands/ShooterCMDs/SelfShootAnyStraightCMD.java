// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCMDs;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShooterCMDs.LowLevelCMDs.SelfShootCurrentAngleCMD;
import frc.robot.commands.ShooterCMDs.LowLevelCMDs.SetAngleDistanceCMD;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SelfShootAnyStraightCMD extends SequentialCommandGroup {
  /** Creates a new ShootAnywhereCMD. */
  public SelfShootAnyStraightCMD(Shooter shooterSubsystem, CommandXboxController controller, DriveTrain driveTrain, Vision visionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetAngleDistanceCMD(shooterSubsystem, visionSubsystem),
      new SelfShootCurrentAngleCMD(shooterSubsystem));
  }
}
