// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NoteState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final FlywheelIO flywheels;

  private double ff;

  private final LeafBlowerIO leafBlower;
  private final FeederIO feeder;
  private DistanceSensorIO dist;
  private NoteState lastNoteState;

  private final FlywheelIOInputsAutoLogged flyInputs = new FlywheelIOInputsAutoLogged();
  private final FeederIOInputsAutoLogged feedInputs = new FeederIOInputsAutoLogged();
  private final DistanceSensorIOInputsAutoLogged sInputs = new DistanceSensorIOInputsAutoLogged();

  private final SimpleMotorFeedforward leftFlywheelFFModel;
  private final SimpleMotorFeedforward rightFlywheelFFModel;
  private final SimpleMotorFeedforward feederFFModel;

  private static final LoggedTunableNumber flywheelkP = new LoggedTunableNumber("flywheelkP");
  private static final LoggedTunableNumber flywheelkI = new LoggedTunableNumber("flywheelkI");
  private static final LoggedTunableNumber flywheelkD = new LoggedTunableNumber("flywheelkD");

  private static final LoggedTunableNumber feederkP = new LoggedTunableNumber("feederkP");
  private static final LoggedTunableNumber feederkI = new LoggedTunableNumber("feederkI");
  private static final LoggedTunableNumber feederkD = new LoggedTunableNumber("feederkD");

  public Shooter(
      FlywheelIO flywheels, FeederIO feeder, DistanceSensorIO dist, LeafBlowerIO leafBlower) {
    switch (Constants.getMode()) {
      case REAL:
        leftFlywheelFFModel = new SimpleMotorFeedforward(0.23, 0.18, 0); // make constant
        rightFlywheelFFModel = new SimpleMotorFeedforward(0.4, 0.5, 0.3);

        feederFFModel = new SimpleMotorFeedforward(0, 0, 0);

        flywheelkP.initDefault(2.056); // make constant
        flywheelkI.initDefault(0);
        flywheelkD.initDefault(0);

        feederkP.initDefault(0.23); // make constant
        feederkI.initDefault(5); // make constant
        feederkD.initDefault(0);
        lastNoteState = NoteState.Init;
        break;
      case REPLAY:
        leftFlywheelFFModel = new SimpleMotorFeedforward(0, 0.03);
        rightFlywheelFFModel = new SimpleMotorFeedforward(0.18, 0.3);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
      case SIM:
        leftFlywheelFFModel = new SimpleMotorFeedforward(0, 0.5);
        rightFlywheelFFModel = new SimpleMotorFeedforward(0.18, 0.3);

        feederFFModel = new SimpleMotorFeedforward(0, 0.5);

        flywheelkP.initDefault(0.4); // make constant
        flywheelkI.initDefault(0);
        flywheelkD.initDefault(0);

        feederkP.initDefault(10);
        feederkI.initDefault(0);
        feederkD.initDefault(0);
        break;
      default:
        leftFlywheelFFModel = new SimpleMotorFeedforward(0, 0.03);
        rightFlywheelFFModel = new SimpleMotorFeedforward(0.18, 0.3);
        feederFFModel = new SimpleMotorFeedforward(0, 0.03);
        break;
    }
    this.flywheels = flywheels;
    flywheels.configurePID(flywheelkP.get(), flywheelkI.get(), flywheelkD.get());

    this.feeder = feeder;
    feeder.configurePID(feederkP.get(), feederkI.get(), feederkD.get());

    this.leafBlower = leafBlower;
    this.dist = dist;
  }

  public void stopFlywheels() {
    flywheels.stop();
  }

  public void stopFeeders() {
    feeder.stop();
  }

  public void setFeedersRPM(double velocityRPM) {
    feeder.setVelocityRPS(velocityRPM / 60.0, feederFFModel.calculate(velocityRPM / 60.));
  }

  public void setFlywheelRPMs(double leftVelocityRPM, double rightVelocityRPM) {

    ff = leftFlywheelFFModel.calculate(rightVelocityRPM / 60.);
    flywheels.setVelocityRPS(
        leftVelocityRPM / 60.,
        rightVelocityRPM / 60.,
        leftFlywheelFFModel.calculate(leftVelocityRPM / 60.),
        rightFlywheelFFModel.calculate(rightVelocityRPM / 60.));
  }

  public void setFlywheelRPMSSource() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      flywheels.setVelocityRPS(
          5000,
          4200,
          leftFlywheelFFModel.calculate(5000 / 60.),
          leftFlywheelFFModel.calculate(4200 / 60.));
    } else {
      flywheels.setVelocityRPS(
          4200,
          5000,
          leftFlywheelFFModel.calculate(5000 / 60.),
          leftFlywheelFFModel.calculate(4200 / 60.));
    }
  }

  public void setFlywheelRPMSAmp() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      flywheels.setVelocityRPS(
          4200,
          5000,
          leftFlywheelFFModel.calculate(4200 / 60.),
          leftFlywheelFFModel.calculate(5000 / 60.));
    } else {
      flywheels.setVelocityRPS(
          5000,
          4200,
          leftFlywheelFFModel.calculate(5000 / 60.),
          leftFlywheelFFModel.calculate(4200 / 60.));
    }
  }

  public double[] getFlywheelVelocitiesRPM() {
    return new double[] {flyInputs.leftVelocityRPM, flyInputs.rightVelocityRPM};
  }

  public double[] getFlywheelErrors() {
    return new double[] {
      flyInputs.leftVelocitySetpointRPM - getFlywheelVelocitiesRPM()[0],
      flyInputs.rightVelocitySetpointRPM - getFlywheelVelocitiesRPM()[1]
    };
  }

  public boolean atFlywheelSetpoints() {
    Logger.recordOutput("err left", getFlywheelErrors()[0]);
    Logger.recordOutput("err right", getFlywheelErrors()[1]);

    Logger.recordOutput(
        "at left",
        Math.abs(getFlywheelErrors()[0]) <= Constants.ShooterConstants.FLYWHEEL_THRESHOLD);
    Logger.recordOutput(
        "at right",
        Math.abs(getFlywheelErrors()[1]) <= Constants.ShooterConstants.FLYWHEEL_THRESHOLD);

    Logger.recordOutput(
        "at yes", flyInputs.leftVelocitySetpointRPM > 0 && flyInputs.rightVelocitySetpointRPM > 0);

    return (Math.abs(getFlywheelErrors()[0]) <= Constants.ShooterConstants.FLYWHEEL_THRESHOLD
            && Math.abs(getFlywheelErrors()[1]) <= Constants.ShooterConstants.FLYWHEEL_THRESHOLD)
        && (flyInputs.leftVelocitySetpointRPM > 0 && flyInputs.rightVelocitySetpointRPM > 0);
  }

  public double getFeederRPM() {
    return feedInputs.feederVelocityRPM;
  }

  public double getFeederError() {
    return 0; // feedInputs.velocitySetpointRPM - getFeederError();
  }

  public boolean atFeederSetpoint() {
    return Math.abs(getFeederError()) <= Constants.ShooterConstants.FEEDER_THRESHOLD;
  }

  public NoteState seesNote() {
    Logger.recordOutput("see note val", "default");
    if ((sInputs.distance > Constants.ShooterConstants.FEEDER_DIST && sInputs.distance < 2150)) {
      Logger.recordOutput("see note val", "sensor");
      lastNoteState = NoteState.SENSOR;
      return NoteState.SENSOR;

    } else if (feedInputs.currentAmps > 13) {
      // } else if (feedInputs.currentAmps > 10000) {
      Logger.recordOutput("see note val", "current");
      lastNoteState = NoteState.CURRENT;
      return NoteState.CURRENT;

    } else {
      Logger.recordOutput("see note val", "no note");
      lastNoteState = NoteState.NO_NOTE;
      return NoteState.NO_NOTE;
    }
  }

  public NoteState getLastNoteState() {
    return lastNoteState;
  }

  public void turnOnFan() {
    leafBlower.setSpeed(-1);
  }

  public void turnOffFan() {
    leafBlower.stop();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Logger.recordOutput("see note", seesNote());
    flywheels.updateInputs(flyInputs);
    feeder.updateInputs(feedInputs);
    dist.updateInputs(sInputs);

    Logger.processInputs("Flywheels", flyInputs);
    Logger.processInputs("Feeder", feedInputs);
    Logger.processInputs("Distance Sensor", sInputs);

    Logger.recordOutput("ffvolt", ff);

    if (feederkP.hasChanged(hashCode())
        || feederkD.hasChanged(hashCode())
        || feederkI.hasChanged(hashCode())) {
      feeder.configurePID(feederkP.get(), feederkI.get(), feederkD.get());
    }

    if (flywheelkP.hasChanged(hashCode())
        || flywheelkI.hasChanged(hashCode())
        || flywheelkD.hasChanged(hashCode())) {
      flywheels.configurePID(flywheelkP.get(), flywheelkI.get(), flywheelkD.get());
    }
  }
}
