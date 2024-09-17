package frc.robot.Intake;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Drivetrain.DriveIO.DriveIOdata;
import frc.robot.Intake.IntakeIO.IntakeIOStats;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOStats stats = new IntakeIOStats();


  public enum State {
    IDLE(0.0),
    INTAKE(IntakeConstants.targetRPMHigh),
    CHARACTERIZING(IntakeConstants.targetRPMLow),
    EJECT(-IntakeConstants.targetRPMEject);

    private final double rpm;
  
    private State (double rpmIn) {
        this.rpm = rpmIn; 
    }

  }

  /* Variable to hold the current state of the state machine  */
  private State currentState = State.IDLE;

  /** Creates a new IntakeSubsystem. */
  public Intake(IntakeIO io) {
    this.io = io; 
  }

  private void flipState(State inState ) {
   System.out.println("Setting state...." + inState.name());
   currentState = inState; 
  }

  /**
   * Set command to Intake
   *
   * @return a command
   */
  public Command intakeCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.INTAKE);
        });
  }

   /**
   * Set command to Idle
   *
   * @return a command
   */
  public Command idleCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          flipState(State.IDLE);
        });
  }

   /**
   * Set Command to Eject
   *
   * @return a command
   */
  public Command ejectCommand() {
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(() -> {flipState(State.EJECT); });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    /* This method will be called once per scheduler run */
    
    //Detwrmine motor speed and process 
    double motorRPM = currentState.rpm;
    io.runVelocity(motorRPM, 0.0); 

    //go update the signal data 
    io.updateStats(stats);
    
    //Update Dashboard with Telementry Data 
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
