// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

/** Add your docs here. */
public class FlywheelIOSim implements FlywheelIO {
  private FlywheelSim left = new FlywheelSim(DCMotor.getKrakenX60(1), 1, 0.004);
  private FlywheelSim right = new FlywheelSim(DCMotor.getKrakenX60(1), 1, 0.004);

  private PIDController leftPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController rightPID = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;

  private double leftFFVolts = 0.0;
  private double leftAppliedVolts = 0.0;
  private double leftVelocitySetpointRPM = 0.0;

  private double rightFFVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private double rightVelocitySetpointRPM = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts =
          MathUtil.clamp(
              leftPID.calculate(left.getAngularVelocityRPM() / 60.) + leftFFVolts, -12.0, 12.0);
      left.setInputVoltage(leftAppliedVolts);
    }

    left.update(Constants.LOOP_PERIOD_SECS);

    inputs.leftVelocitySetpointRPM = leftVelocitySetpointRPM;
    inputs.leftVelocityRPM = left.getAngularVelocityRPM();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = left.getCurrentDrawAmps();

    if (closedLoop) {
      rightAppliedVolts =
          MathUtil.clamp(
              rightPID.calculate(right.getAngularVelocityRPM() / 60.) + rightFFVolts, -12.0, 12.0);
      right.setInputVoltage(rightAppliedVolts);
    }

    right.update(Constants.LOOP_PERIOD_SECS);

    inputs.rightVelocitySetpointRPM = rightVelocitySetpointRPM;
    inputs.rightVelocityRPM = right.getAngularVelocityRPM();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = right.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;

    leftAppliedVolts = 0.0;
    left.setInputVoltage(volts);

    rightAppliedVolts = 0.0;
    right.setInputVoltage(volts);
  }

  @Override
  public void setVelocityRPS(
      double leftVelocityRPS, double rightVelocityRPS, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;

    this.leftVelocitySetpointRPM = leftVelocityRPS * 60;
    leftPID.setSetpoint(leftVelocityRPS);
    this.leftFFVolts = leftFFVolts;

    this.rightVelocitySetpointRPM = rightVelocityRPS * 60;
    rightPID.setSetpoint(rightVelocityRPS);
    this.rightFFVolts = rightFFVolts;
  }

  @Override
  public void stop() {
    leftVelocitySetpointRPM = 0;
    rightVelocitySetpointRPM = 0;

    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    leftPID.setPID(kP, kI, kD);
    rightPID.setPID(kP, kI, kD);
  }
}
