// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCMDs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveTrain;

public class DriveCMD extends Command {

  private DriveTrain drive;
  
  private CommandXboxController drivecontroller;
  
  private boolean fieldOriented;
  private boolean rateLimited;
  public DriveCMD(DriveTrain drive, CommandXboxController controller, boolean fieldOriented, boolean rateLimted) {
      this.drive = drive;
      this. rateLimited = rateLimted;

      this.fieldOriented = fieldOriented;
      this.drivecontroller = controller;
      addRequirements(drive);
  }

  @Override
  public void execute() {
      drive.drive(
          MathUtil.applyDeadband(drivecontroller.getLeftY(), 0.3),
          MathUtil.applyDeadband(drivecontroller.getLeftX(), 0.3),
          MathUtil.applyDeadband(-drivecontroller.getRightX(), 0.3),
          fieldOriented, rateLimited);
  }
}
