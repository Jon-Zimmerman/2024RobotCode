package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface LeafBlowerIO {
  @AutoLog
  static class LeafBlowerIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  public default void updateInputs(LeafBlowerIOInputs inputs) {}

  public default void setSpeed(double percentSpeed) {}

  public default void stop() {}
}
