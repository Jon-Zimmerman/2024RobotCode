package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    double elevatorPosition = 0;
    double elevatorVelocity = 0;
    double currentAmps = 0;
    double appliedVolts = 0;
    double positionSetpoint = 0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void runCharacterization(double volts) {}

  public default void setPositionSetpoint(double position, double ffVolts) {}

  public default void stop() {}

  public default void setVoltage(double volts) {}

  public default void configurePID(double kP, double kI, double kD) {}
}
