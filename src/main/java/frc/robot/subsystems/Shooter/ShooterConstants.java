package frc.robot.subsystems.Shooter;

public class ShooterConstants {
    
    public static final int kIntakeMotorID = 0;
    public static final double kReduction = (1.0 / 2.0);
    public static final double kMaxAccelerationRpmPerSec = 9000.0; 
    public static final Gains gains = new Gains(0.05, 0.0, 0.0, 0.1, 0.00103, 0.0);

    
    
    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {} 
}
