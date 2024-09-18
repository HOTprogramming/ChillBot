// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOStats;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOStats stats = new ShooterIOStats();
  
  private static final double _SHOOT = 2000.0; 
  private static final double _IDLE = 1000.0;
  private static final double _OFF = 0.0; 

  public enum State {
    SHOOT(_SHOOT),
    IDLE(_IDLE),
    OFF(_OFF);

    private final double rpm;
  
    private State (double rpmIn) {
        this.rpm = rpmIn; 
    }

  }

  private State currentState = State.OFF;


  public ShooterSubsystem(ShooterIO io) {
    this.io = io; 
  }

  public Command shootCommand() {
    
    return runOnce(() -> {currentState = State.SHOOT;});
  }

  public Command idleCommand() {
  
    return runOnce(() -> {currentState = State.IDLE;});
  }

    public Command OFFCommand() {
  
    return runOnce(() -> {currentState = State.OFF;});
  }

  public boolean exampleCondition() {

    return false;
  }

  @Override
  public void periodic() {

    double motorRPM = currentState.rpm;
    if(currentState == State.IDLE || currentState == State.SHOOT){
    io.runVelocity(motorRPM, 0.0); 
    }
    else{
      
    }

    io.updateStats(stats);
    
    UpdateTelemetry();
  
   
  }

  private void UpdateTelemetry() {
    SmartDashboard.putNumber("Applied Volts:",stats.AppliedVolts);
    SmartDashboard.putNumber("Velocity RPM:",stats.VelocityRpm);
    SmartDashboard.putNumber("Position (rads):",stats.PositionRads);
    SmartDashboard.putNumber("Supply Current(Amps):",stats.SupplyCurrentAmps);
    SmartDashboard.putString("StateName", currentState.name());
    SmartDashboard.putNumber("Goal RPM", currentState.rpm);
    SmartDashboard.putNumber("Applied Volts:",stats.AppliedVolts); 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}