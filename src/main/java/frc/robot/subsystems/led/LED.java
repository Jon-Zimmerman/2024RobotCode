// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_STATE;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class LED extends SubsystemBase {
  private final LED_IO led;
  private final LED_IOInputsAutoLogged lInputs = new LED_IOInputsAutoLogged();

  public LED(LED_IO led) {
    this.led = led;
  }

  @Override
  public void periodic() {
    led.updateInputs(lInputs);

    setState(lInputs.ledState);

    Logger.processInputs("LED Inputs", lInputs);
  }

  public void noBumpersPressed() {
    led.noBumpersPressed();
  }

  public void setState(LED_STATE state) {
    led.setLEDState(state);
    Logger.recordOutput("Set State", state);
  }
}
