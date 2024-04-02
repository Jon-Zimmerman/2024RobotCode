// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  private final Intake intake;

  private final LED led;
  private final Shooter shooter;

  public IntakeNote(Intake intake, Shooter shooter, LED led) {
    this.intake = intake;
    this.shooter = shooter;
    this.led = led;

    addRequirements(intake, shooter, led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runRollers(12);
    shooter.setFeedersRPM(500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFeedersRPM(0);
    intake.stopRollers();
    // if (shooter.seesNote()) ;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.seesNote();
  }
}
