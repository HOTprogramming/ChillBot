// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmIO.ArmIOStats;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOStats stats = new ArmIOStats();
  
  //lowest value is 0.984, highest value is 1.17
  private static final double ZERO_POS = 1.0;
  private static final double BATTER_POS = 1.05;
  private static final double PROTECTED_POS = 1.1;

  public enum State {
    ZERO(ZERO_POS),
    BATTER(BATTER_POS),
    PROTECTED(PROTECTED_POS);

    private final double armPos;
  
    private State (double armPosIn) {
        this.armPos = armPosIn; 
    }

  }

  private State currentState = State.ZERO;

  public ArmSubsystem(ArmIO io) {
    this.io = io; 
  }



  public Command protectCommand() {
  return runOnce(() -> {currentState = State.PROTECTED; });
  }

  public Command zeroCommand() {
   return runOnce(() -> {currentState = State.ZERO;});
  }

  public Command batterCommand() {
    return runOnce(() -> {currentState = State.BATTER; });
  }

  public boolean armExampleCondition() {
    return false;
  }

  @Override
  public void periodic() {

    double armControl = currentState.armPos;
    io.setArmMotorControl(armControl);
    io.updateArmStats(stats);
    
    UpdateTelemetry();
  
   
  }

  private void UpdateTelemetry() {
    SmartDashboard.putNumber("Cancoder Position:",stats.cancoderPosition);
    SmartDashboard.putNumber("Cancoder Velocity",stats.cancoderVelocity);
    SmartDashboard.putNumber("Arm Position",stats.armPosition);
    SmartDashboard.putNumber("Arm Velocity",stats.armVelocity);
    SmartDashboard.putNumber("Arm Follower Velocity",stats.armFollowerVelocity);
    SmartDashboard.putNumber("Arm Follower Position",stats.armFollowerPosition);
    SmartDashboard.putString("Arm State", currentState.name());
    SmartDashboard.putNumber("Goal Arm Pos", currentState.armPos);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}