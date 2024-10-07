package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface DistanceSensorIO {
  @AutoLog
  public static class DistanceSensorIOInputs {
    public double distance = 0;
    public int sustain = 0;
  }

  public default void updateInputs(DistanceSensorIOInputs inputs) {}

  public default void increaseSustain() {}

  public default void resetSustain() {}
}
