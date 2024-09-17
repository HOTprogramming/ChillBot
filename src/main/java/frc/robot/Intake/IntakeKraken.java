package frc.robot.Intake;

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

public class IntakeKraken implements IntakeIO {
  // Hardware
  private final TalonFX intakeTalon;

  // Status Signals
  private final StatusSignal<Double> Position;
  private final StatusSignal<Double> Velocity;
  private final StatusSignal<Double> AppliedVolts;
  private final StatusSignal<Double> SupplyCurrent;
  private final StatusSignal<Double> TorqueCurrent;
  private final StatusSignal<Double> TempCelsius;

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
  //private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);

  public IntakeKraken() {
    intakeTalon = new TalonFX(IntakeConstants.kInnerIntakeMotorID, IntakeConstants.CANBUS_NAME);


    // Apply configs
    intakeTalon.getConfigurator().apply(IntakeConstants.innerRollerConfig, 1.0);

    // Set inverts
    intakeTalon.setInverted(true);

    // Set signals
    Position = intakeTalon.getPosition();
    Velocity = intakeTalon.getVelocity();
    AppliedVolts = intakeTalon.getMotorVoltage();
    SupplyCurrent = intakeTalon.getSupplyCurrent();
    TorqueCurrent = intakeTalon.getTorqueCurrent();
    TempCelsius = intakeTalon.getDeviceTemp();

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
  public void updateStats(IntakeIOStats stats) {
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
    intakeTalon.setControl(voltageControl.withOutput(Volts));
  }

  @Override
  public void stop() {
    intakeTalon.setControl(neutralControl);
  }

  @Override
  public void runVelocity(double Rpm, double Feedforward) {
    if (Rpm != 0) {
      intakeTalon.setControl(
        velocityControl.withVelocity(Rpm / 60.0));
    } else {
      intakeTalon.setControl(neutralControl); // because TorqueFOC tuning doesnt like a 0 command
    }
    
  }

  @Override
  /**
   * @param kp P in PID
   * @param kV Amperage needed to sustain a high setpoint (kinda)
   * @param kS Amperage needed to overcome static friction (kinda)
   * kv ks and kp are the only values needed to get a good tune on any motor to attain a velocity
   */
  public void setPID(double kP, double kV, double kS) {
    intakeTalon.getConfigurator().apply(new Slot0Configs()
      .withKP(kP)
      .withKV(kV)
      .withKS(kS));
  }

  @Override
  public void runCharacterization(double input) {
    intakeTalon.setControl(voltageControl.withOutput(input));
  }

}
