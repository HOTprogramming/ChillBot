package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    private static RobotType robotType = RobotType.COMPBOT;

    public enum RobotType {
        SIMBOT,
        DEVBOT,
        COMPBOT
      }

    public static RobotType getRobot() {
        return robotType;
    }
}
