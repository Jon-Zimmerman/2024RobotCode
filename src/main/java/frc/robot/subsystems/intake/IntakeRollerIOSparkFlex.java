package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class IntakeRollerIOSparkFlex implements IntakeRollerIO {
  private final CANSparkFlex rollers;
  private final SparkPIDController pid;

  private double velocitySetpoint = 0;

  public IntakeRollerIOSparkFlex(int id) {
    rollers = new CANSparkFlex(id, MotorType.kBrushless);
    rollers.restoreFactoryDefaults();
    rollers.setIdleMode(IdleMode.kCoast);
    rollers.setInverted(true);

    rollers.setSmartCurrentLimit(Constants.IntakeConstants.CURRENT_LIMIT);
    rollers.setCANTimeout(250);
    rollers.burnFlash();

    pid = rollers.getPIDController();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.rollerRotations = rollers.getEncoder().getPosition();
    inputs.rollerVelocityRPM = rollers.getEncoder().getVelocity() / 60;

    inputs.appliedVolts = rollers.getAppliedOutput();
    inputs.currentAmps = rollers.getOutputCurrent();
    inputs.velocitySetpoint = velocitySetpoint;
  }

  @Override
  public void runCharacterization(double volts) {
    rollers.setVoltage(volts);
  }

  @Override
  public void setVelocityRPM(double velocity, double ffVolts) {
    this.velocitySetpoint = velocity;
    // pid.setReference(velocity, ControlType.kVelocity, 0, ffVolts, ArbFFUnits.kVoltage);
    rollers.setVoltage(9);
  }

  @Override
  public void stop() {
    rollers.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }
}
