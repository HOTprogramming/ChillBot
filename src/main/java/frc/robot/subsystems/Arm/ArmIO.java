package frc.robot.subsystems.Arm;

public interface ArmIO {
        
        class ArmIOStats {
                public boolean armMotorConnected = true;
                public boolean f_fusedSensorOutOfSync;
                public boolean sf_fusedSensorOutOfSync;
                public boolean f_remoteSensorInvalid;
                public boolean sf_remoteSensorInvalid;
                public double armPosition;
                public double armVelocity;
                public double armFollowerPosition;
                public double armFollowerVelocity;
                public double cancoderPosition;
                public double cancoderVelocity;
                public double armRotorPos;
                public double armFollowerRotorPos;
              }

        /** Update stats */
        default void updateArmStats(ArmIOStats stats) {}

        //apply motion magic control mode
        default void setArmMotorControl(double commandedPosition){ 
        }
       
}