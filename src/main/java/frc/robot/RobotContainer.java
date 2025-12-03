// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class RobotContainer {
    // Subsystem definitions
    private final ElevatorSubsystem elevator;

    // Controller definitions
    private final CommandXboxController driverController = new CommandXboxController(0);

    // Dashboard inputs
    //   private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                elevator = new ElevatorSubsystem(new ElevatorIO() {});
                break;
            case SIM:
                elevator = new ElevatorSubsystem(new ElevatorIOSim());
                break;
            default:
                elevator = new ElevatorSubsystem(new ElevatorIO() {});
                break;
        }

        // Set up auto routines
        // autoChooser = new LoggedDashboardChooser<>("Auto Choices",
        // AutoBuilder.buildAutoChooser());

        // Configure the button bindings
        configureBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        driverController
                .button(1)
                .whileTrue(elevator.runGoalPositionCommandFactory(() -> Meters.of(1)));
        driverController
                .button(4)
                .whileTrue(elevator.runGoalVelocityCommandFactory(() -> MetersPerSecond.of(1)));
        elevator.setDefaultCommand(
                elevator.runGoalPositionCommandFactory(
                        () -> Meters.of(1.5 + 1.5 * driverController.getRawAxis(0))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
