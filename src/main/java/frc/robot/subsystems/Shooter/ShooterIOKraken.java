package frc.robot.subsystems.Shooter;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class ShooterIOKraken  implements ShooterIO {
  // Hardware
  private final TalonFX shooter1;

  // Status Signals
  private final StatusSignal<Double> Position;
  private final StatusSignal<Double> Velocity;
  private final StatusSignal<Double> AppliedVolts;
  private final StatusSignal<Double> SupplyCurrent;
  private final StatusSignal<Double> TorqueCurrent;
  private final StatusSignal<Double> TempCelsius;

  // Control
  private final Slot0Configs controllerConfig = new Slot0Configs();
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  //private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public ShooterIOKraken() {
    shooter1 = new TalonFX(ShooterConstants.kIntakeMotorID, "drivetrain");
      

    // General config
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    //*Cory Added Does not apply to Torque control on Voltage/Velocity Control*/
    config.CurrentLimits.StatorCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimitEnable = true; 
    //*End of Cory Added  */
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.kReduction;

    // Controller config;
    controllerConfig.kP = ShooterConstants.gains.kP();
    controllerConfig.kI = ShooterConstants.gains.kI();
    controllerConfig.kD = ShooterConstants.gains.kD();
    controllerConfig.kS = ShooterConstants.gains.kS();
    controllerConfig.kV = ShooterConstants.gains.kV();
    controllerConfig.kA = ShooterConstants.gains.kA();

    // Apply configs
    shooter1.getConfigurator().apply(config, 1.0);
    shooter1.getConfigurator().apply(controllerConfig, 1.0);

    // Set inverts
    shooter1.setInverted(true);

    // Set signals
    Position = shooter1.getPosition();
    Velocity = shooter1.getVelocity();
    AppliedVolts = shooter1.getMotorVoltage();
    SupplyCurrent = shooter1.getSupplyCurrent();
    TorqueCurrent = shooter1.getTorqueCurrent();
    TempCelsius = shooter1.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        Position,
        Velocity,
        AppliedVolts,
        SupplyCurrent,
        TorqueCurrent,
        TempCelsius);
  }

  @Override
  public void updateStats(ShooterIOStats stats) {
    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                Position,
                Velocity,
                AppliedVolts,
                SupplyCurrent,
                TorqueCurrent,
                TempCelsius)
            .isOK();

    stats.PositionRads = Units.rotationsToRadians(Position.getValueAsDouble());
    stats.VelocityRpm = Velocity.getValueAsDouble() * 60.0;
    stats.AppliedVolts = AppliedVolts.getValueAsDouble();
    stats.SupplyCurrentAmps = SupplyCurrent.getValueAsDouble();
    stats.TorqueCurrentAmps = TorqueCurrent.getValueAsDouble();
    stats.TempCelsius = TempCelsius.getValueAsDouble();

  }

  @Override
  public void runVolts(double Volts) {
    shooter1.setControl(voltageControl.withOutput(Volts));
  }

  @Override
  public void stop() {
    shooter1.setControl(neutralControl);
  }

  @Override
  public void runVelocity(double Rpm, double Feedforward) {
    shooter1.setControl(
        velocityControl.withVelocity(Rpm / 60.0));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controllerConfig.kP = kP;
    controllerConfig.kI = kI;
    controllerConfig.kD = kD;
    shooter1.getConfigurator().apply(controllerConfig);
  }

  @Override
  public void runCharacterization(double input) {
    shooter1.setControl(voltageControl.withOutput(input));
  }

}