package frc.robot.Intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public interface IntakeIO {
    class IntakeIOStats {
        public boolean MotorConnected = true;    
        public double PositionRads = 0.0;
        public double VelocityRpm = 0.0;
        public double AppliedVolts = 0.0;
        public double SupplyCurrentAmps = 0.0;
        public double TorqueCurrentAmps = 0.0;
        public double TempCelsius = 0.0;
      }

/** Update stats */
default void updateStats(IntakeIOStats stats) {}

/** Run motor at voltage */
default void runVolts(double Volts) {}

/** Stop Intake */
default void stop() {}

/** Run motor at velocity in rpm */
default void runVelocity(double Rpm, double Feedforward) {}

/** Config PID values for motor */
default void setPID(double kP, double kI, double kD) {}

/** Config FF values for motor */
default void setFF(double kS, double kV, double kA) {}

/** Run Characterization for motor at voltage */
default void runCharacterization(double input) {}
}
