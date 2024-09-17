// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drivetrain.Drive;
import frc.robot.Drivetrain.DriveIO;
import frc.robot.Drivetrain.DriveKraken;
import frc.robot.Drivetrain.DriveSim;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeKraken;
import frc.robot.Intake.IntakeSim;

public class RobotContainer {
  public Drive drivetrain;
  public Intake intake;

  private final CommandXboxController driver = new CommandXboxController(0);

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        DriveKraken krakenDrive = new DriveKraken();
        IntakeKraken intakeKraken = new IntakeKraken();

        this.drivetrain = new Drive(krakenDrive);
        this.intake = new Intake(intakeKraken);
      }
      case DEVBOT -> {}
      case SIMBOT -> {
        DriveSim simIO = new DriveSim();
        IntakeSim intakeSim = new IntakeSim();

        this.drivetrain = new Drive(simIO);
        this.intake = new Intake(intakeSim);
      }
    }
    drivetrain.setGains();

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand
      (drivetrain.run(
        () -> 
          {
            drivetrain.teleopDrive(
              Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0, 
              Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0, 
              Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
          }
      ));

    driver.a().whileTrue(intake.intakeCommand()).whileFalse(intake.idleCommand());
    driver.b().whileTrue(intake.ejectCommand()).whileFalse(intake.idleCommand());
    driver.x().whileTrue(intake.idleCommand());
  
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
