package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FeederIOSim implements FeederIO {
  private final DCMotor motor = DCMotor.getKrakenX60(1);
  private DCMotorSim sim = new DCMotorSim(motor, 1, 0.7);
  private PIDController pid = new PIDController(0, 0, 0);

  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    appliedVolts =
        MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM()) + ffVolts, -12.0, 12.0);

    sim.setInputVoltage(appliedVolts);

    sim.update(Constants.LOOP_PERIOD_SECS);

    inputs.feederVelocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void runCharacterization(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void setVelocityRPS(double velocity, double ffVolts) {
    this.ffVolts = ffVolts;
    pid.setSetpoint(velocity);
  }

  @Override
  public void stop() {
    setVelocityRPS(0, 0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
