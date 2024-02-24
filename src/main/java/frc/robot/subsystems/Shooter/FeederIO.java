package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double feederVelocityRPM = 0;
    public double feederRotations;
    public double currentAmps = 0;
    public double appliedVolts = 0;
    public double velocitySetpointRPS = 0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void runCharacterization(double volts) {}

  public default void setVelocityRPS(double velocity, double ffVolts) {}

  public default void stop() {}

  public default void configurePID(double kP, double kI, double kD) {}
}
