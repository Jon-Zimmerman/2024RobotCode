// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorExtenderIO;
import frc.robot.subsystems.Elevator.ElevatorExtenderIOSim;
import frc.robot.subsystems.Elevator.ElevatorExtenderIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorPivotIO;
import frc.robot.subsystems.Elevator.ElevatorPivotIOSim;
import frc.robot.subsystems.Elevator.ElevatorPivotIOTalonFX;
import frc.robot.subsystems.ExampleSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final Elevator elevator;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
        // Real robot, instantiate hardware IO implementations
      case REAL:
        elevator =
            new Elevator(
                new ElevatorPivotIOTalonFX(
                    Constants.ELEVATOR_PIVOT_ID, Constants.CANCODER_CANBUS_ID),
                new ElevatorExtenderIOTalonFX(Constants.ELEVATOR_EXTENDER_ID));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorPivotIOSim(), new ElevatorExtenderIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        elevator = new Elevator(new ElevatorPivotIO() {}, new ElevatorExtenderIO() {});
        break;
    }

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // m_driverController.b().whileTrue(Commands.startEnd(() -> elevator.setExtenderGoal(1.5), elevator::elevatorStop, elevator));
    // m_driverController.a().whileTrue(Commands.startEnd(() -> elevator.setExtenderGoal(3), elevator::elevatorStop, elevator));
    // m_driverController.y().whileTrue(Commands.startEnd(() -> elevator.setExtenderGoal(0), elevator::elevatorStop, elevator));

    m_driverController.b().whileTrue(Commands.startEnd(() -> elevator.setPivotGoal(Math.PI/3), elevator::pivotStop, elevator));
    m_driverController.a().whileTrue(Commands.startEnd(() -> elevator.setPivotGoal(Math.PI), elevator::pivotStop, elevator));
    m_driverController.y().whileTrue(Commands.startEnd(() -> elevator.setPivotGoal(0), elevator::pivotStop, elevator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
