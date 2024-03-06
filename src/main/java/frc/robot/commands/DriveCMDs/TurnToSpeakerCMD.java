// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class TurnToSpeakerCMD extends Command {

  private DriveTrain drive;
  private Vision vision;
  private CommandXboxController controller;
  
  
  private PIDController pidController;
  public TurnToSpeakerCMD(CommandXboxController controller, DriveTrain drive, Vision vision) {
      this.drive = drive;
      this.vision = vision;
      this.controller = controller;

      pidController = new PIDController(.0075, 0, 0.0005);
      addRequirements(drive);
  }

  @Override
  public void execute() {
      drive.drive(
          MathUtil.applyDeadband(controller.getLeftY(), 0.3),
          MathUtil.applyDeadband(controller.getLeftX(), 0.3),
          pidController.calculate(vision.getX()),
          true, false);

    SmartDashboard.putNumber("test lime", vision.getX());
  }
}