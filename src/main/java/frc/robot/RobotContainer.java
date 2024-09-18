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
import frc.robot.Feeder.Feeder;
import frc.robot.Feeder.FeederKraken;
import frc.robot.Feeder.FeederSim;
import frc.robot.Intake.Intake;
import frc.robot.Intake.IntakeKraken;
import frc.robot.Intake.IntakeSim;
import frc.robot.subsystems.Arm.ArmIOKraken;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterIOKraken;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RobotContainer {
  public Drive drivetrain;
  public Intake intake;
  public Feeder feeder;


public class RobotContainer {
  public Drive drivetrain;
  private ArmSubsystem m_armSubsystem;
  private ShooterSubsystem m_shooterSubsystem;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    switch (Constants.getRobot()) {
      case COMPBOT -> {
        DriveKraken krakenDrive = new DriveKraken();
        IntakeKraken intakeKraken = new IntakeKraken();
        FeederKraken feederKraken = new FeederKraken();

        this.drivetrain = new Drive(krakenDrive);
        this.intake = new Intake(intakeKraken);
        this.feeder = new Feeder(feederKraken);
        this.drivetrain = new Drive(krakenIO);

        m_armSubsystem = new ArmSubsystem((new ArmIOKraken()));
        m_shooterSubsystem = new ShooterSubsystem((new ShooterIOKraken()));
      }
      case DEVBOT -> {}
      case SIMBOT -> {
        DriveSim simIO = new DriveSim();
        IntakeSim intakeSim = new IntakeSim();
        FeederSim feederSim = new FeederSim();

        this.drivetrain = new Drive(simIO);
        this.intake = new Intake(intakeSim);
        this.feeder = new Feeder(feederSim);
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

    driver.y().whileTrue(feeder.shoot()).whileFalse(feeder.idleCommand());
    driver.a().whileTrue(Commands.parallel(intake.intakeCommand(), feeder.FeederCommand())).whileFalse(Commands.parallel(intake.idleCommand(), feeder.idleCommand()));
    driver.b().whileTrue(intake.ejectCommand()).whileFalse(intake.idleCommand());
    driver.x().whileTrue(intake.idleCommand());
    
  
    operator.b().whileTrue(Commands.parallel(m_armSubsystem.protectCommand(), m_shooterSubsystem.shootCommand()));
    operator.y().whileTrue(Commands.parallel(m_armSubsystem.batterCommand(), m_shooterSubsystem.shootCommand()));
    operator.a().whileTrue(Commands.parallel(m_armSubsystem.zeroCommand(), m_shooterSubsystem.idleCommand()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
