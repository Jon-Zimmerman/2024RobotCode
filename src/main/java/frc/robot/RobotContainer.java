// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetExtenderTarget;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOSparkFlex;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOSpark;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.DistanceSensorIO;
import frc.robot.subsystems.shooter.FeederIOSim;
import frc.robot.subsystems.shooter.FeederIOTalonFX;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private Shooter shooter;
  private Elevator elevator;
  private LED led;
  private Pivot pivot;

  private final CommandXboxController controller = new CommandXboxController(0);
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedDashboardNumber flywheelSpeed = new LoggedDashboardNumber("flywheelSpeed");
  private final LoggedDashboardNumber feedSpeed = new LoggedDashboardNumber("feedSpeed");

  private final LoggedDashboardNumber elevatorDistance =
      new LoggedDashboardNumber("elevatorDistance");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        intake = new Intake(new IntakeRollerIOSparkFlex(RobotMap.IntakeIDs.ROLLERS));
        shooter =
            new Shooter(
                new FlywheelIOTalonFX(
                    RobotMap.ShooterIDs.FLYWHEEL_LEFT, RobotMap.ShooterIDs.FLYWHEEL_RIGHT),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER),
                new DistanceSensorIO() {});
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(RobotMap.ElevatorIDs.LEFT, RobotMap.ElevatorIDs.RIGHT));
        // pivot = new Pivot(new PivotIOTalonFX(RobotMap.PivotIDs.LEFT,RobotMap.PivotIDs.RIGHT,
        // RobotMap.PivotIDs.GYRO));
        led = new LED(new LED_IOSpark(RobotMap.LEDIDs.CHANNEL));
        break;
      case REPLAY:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeRollerIOSim());
        shooter = new Shooter(new FlywheelIOSim(), new FeederIOSim(), new DistanceSensorIO() {});
        elevator = new Elevator(new ElevatorIOSim());
        pivot = new Pivot(new PivotIOSim());
        led = new LED(new LED_IO() {});
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeRollerIOSim());
        shooter = new Shooter(new FlywheelIOSim(), new FeederIOSim(), new DistanceSensorIO() {});
        elevator = new Elevator(new ElevatorIOSim());
        pivot = new Pivot(new PivotIOSim());
        led = new LED(new LED_IO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        intake = new Intake(new IntakeRollerIOSparkFlex(RobotMap.IntakeIDs.ROLLERS));
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter =
            new Shooter(
                new FlywheelIOTalonFX(
                    RobotMap.ShooterIDs.FLYWHEEL_LEFT, RobotMap.ShooterIDs.FLYWHEEL_RIGHT),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER),
                new DistanceSensorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        pivot = new Pivot(new PivotIO() {});
        led = new LED(new LED_IO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // // Intake SysID Routines
    // autoChooser.addDefaultOption(
    //     "Intake SysID (Dynamic Forward)", intake.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addDefaultOption(
    //     "Intake SysID (Quasistatic Reverse)",
    //     intake.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addDefaultOption(
    //     "Intake SysID (Quasistatic Forward)",
    //     intake.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addDefaultOption(
    //     "Intake SysID (Dynamic Reverse)", intake.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // // Flywheel SysID Routines
    // autoChooser.addDefaultOption(
    //     "Flywheel SysID (Dynamic Forward)",
    //     shooter.flywheelSysIDDynamiCommand(SysIdRoutine.Direction.kForward));
    // autoChooser.addDefaultOption(
    //     "Flywheel SysID (Quasistatic Reverse)",
    //     shooter.flywheelSysIDQuasistic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addDefaultOption(
    //     "Flywheel SysID (Quasistatic Forward)",
    //     shooter.flywheelSysIDQuasistic(SysIdRoutine.Direction.kForward));
    // autoChooser.addDefaultOption(
    //     "Flywheel SysID (Dynamic Reverse)",
    //     shooter.flywheelSysIDDynamiCommand(SysIdRoutine.Direction.kReverse));

    // Feeder SysID Routines
    autoChooser.addDefaultOption(
        "Feeder SysID (Dynamic Forward)",
        shooter.feedSysIDDynamiCommand(SysIdRoutine.Direction.kForward));
    autoChooser.addDefaultOption(
        "Feeder SysID (Quasistatic Reverse)",
        shooter.feedSysIDQuasistic(SysIdRoutine.Direction.kReverse));
    autoChooser.addDefaultOption(
        "Feeder SysID (Quasistatic Forward)",
        shooter.feedSysIDQuasistic(SysIdRoutine.Direction.kForward));
    autoChooser.addDefaultOption(
        "Feeder SysID (Dynamic Reverse)",
        shooter.feedSysIDDynamiCommand(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // controller.a().whileTrue(intake.sysIdQuasistatic(Direction.kForward));
    // controller.a().onTrue(new SetPivotTarget(20, elevator));
    // controller.a().onFalse(new InstantCommand(elevator::pivotStop, elevator));

    controller.a().onTrue(new SetExtenderTarget(elevatorDistance.get(), elevator));
    controller.a().onFalse(new InstantCommand(elevator::elevatorStop, elevator));

    controller.b().onTrue(new SetExtenderTarget(0.1, elevator));
    controller.b().onFalse(new InstantCommand(elevator::elevatorStop, elevator));

    // controller.a().onTrue(new InstantCommand(() -> shooter.setFlywheelRPMs(300, 300), shooter));
    // controller.a().onFalse(new InstantCommand(shooter::stopShooterMotors, shooter));

    // controller.b().onTrue(new InstantCommand(() -> shooter.runFeeders(1500), shooter));
    // controller.b().onFalse(new InstantCommand(shooter::stopFeeders, shooter));

    // controller.a().onTrue(new SetExtenderTarget(0, elevator));
    // controller.a().onFalse(new InstantCommand(elevator::elevatorStop, elevator));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    // controller.a().whileTrue(new TurnToSpeaker(drive, controller));

    // controller.a().onTrue(new InstantCommand(() -> intake.runRollers(3000)));
    // controller.a().onFalse(new InstantCommand(() -> intake.stopRollers()));

    // controller
    //     .x()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> shooter.setFlywheelRPMs(flywheelSpeed.get(), flywheelSpeed.get())));
    // controller.x().onFalse(new InstantCommand(() -> shooter.stopShooterMotors()));

    // controller.a().onTrue(new InstantCommand(() -> shooter.setFeedersRPM(feedSpeed.get())));
    // controller.a().onFalse(new InstantCommand(() -> shooter.stopFeeders()));

    // controller.leftBumper().onTrue(new InstantCommand(() -> intake.runRollers(1000)));
    // controller.leftBumper().onFalse(new InstantCommand(() -> intake.stopRollers()));
    // controller.rightBumper().onTrue(Commands.run(() -> elevator.setPivotGoal(Math.PI),
    // elevator));
    // controller.y().onTrue(Commands.run(() -> elevator.setPivotGoal(0), elevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
