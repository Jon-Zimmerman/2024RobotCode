package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LeafBlowerIOTalonSRX implements LeafBlowerIO {

  private final TalonSRX leafBlower;

  public LeafBlowerIOTalonSRX(int id) {

    leafBlower = new TalonSRX(id);
  }

  @Override
  public void updateInputs(LeafBlowerIOInputs inputs) {
    inputs.appliedVolts = leafBlower.getMotorOutputVoltage();
    inputs.currentAmps = leafBlower.getStatorCurrent();
  }

  @Override
  public void setSpeed(double percentSpeed) {
    leafBlower.set(ControlMode.PercentOutput, percentSpeed);
  }

  @Override
  public void stop() {
    leafBlower.set(ControlMode.PercentOutput, 0);
  }
}
