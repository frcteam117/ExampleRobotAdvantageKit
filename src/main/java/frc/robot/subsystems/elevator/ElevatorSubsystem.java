// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  /** Interface to control the elevator's motors */
  private final ElevatorIO io;

  /** Inputs of the elevator's motors */
  private final ElevatorIOInputs ioInputs = new ElevatorIOInputs();

  private TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity_mps, maxAcceleration_mps2));

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(ioInputs);
    Logger.processInputs(name, ioInputs);
    // Logger.recordOutput(name + "/Inputs", ioInputs);
  }

  // Voltage methods

  /** Sets the elevator to the supplied voltage. */
  public void setVoltage(double voltage_V) {
    io.setVoltage(voltage_V);
  }

  /** Sets the elevator to the supplied voltage. */
  public Command setVoltageCommandFactory(DoubleSupplier voltageSupplier_V) {
    return this.runOnce(
        () -> {
          setVoltage(voltageSupplier_V.getAsDouble());
        });
  }

  /** Runs the elevator at the supplied voltage. */
  public Command runVoltageCommandFactory(DoubleSupplier voltageSupplier_V) {
    return this.run(
        () -> {
          setVoltage(voltageSupplier_V.getAsDouble());
        });
  }

  // Position methods

  /** Runs the elevator to and holds at the supplied height. */
  public void setGoalPosition(double height_m) {
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity_mps, maxAcceleration_mps2));
    TrapezoidProfile.State nextState = calculateNextState(new TrapezoidProfile.State(height_m, 0));
    io.setNextState(nextState.position, nextState.velocity);
  }

  /** Runs the elevator to and holds at the supplied height. */
  public Command setGoalPositionCommandFactory(DoubleSupplier heightSupplier_m) {
    return this.runOnce(
        () -> {
          setGoalPosition(heightSupplier_m.getAsDouble());
        });
  }

  /** Runs the elevator to and holds at the supplied height. */
  public Command runGoalPositionCommandFactory(DoubleSupplier heightSupplier_m) {
    return this.run(
        () -> {
          setGoalPosition(heightSupplier_m.getAsDouble());
        });
  }

  // Velocity methods

  /** Runs the elevator to and holds at the supplied velocity. */
  public void setGoalVelocity(double velocity_mps) {
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity_mps, maxAcceleration_mps2));
    TrapezoidProfile.State nextState =
        calculateNextState(
            new TrapezoidProfile.State(
                velocity_mps >= 0 ? maxPosition_m : minPosition_m, velocity_mps));
    io.setNextState(nextState.position, nextState.velocity);
  }

  /** Runs the elevator to and holds at the supplied velocity. */
  public Command setGoalVelocityCommandFactory(DoubleSupplier velocitySupplier_mps) {
    return this.runOnce(
        () -> {
          setGoalVelocity(velocitySupplier_mps.getAsDouble());
        });
  }

  /** Runs the elevator to and holds at the supplied velocity. */
  public Command runGoalVelocityCommandFactory(DoubleSupplier velocitySupplier_mps) {
    return this.run(
        () -> {
          setGoalVelocity(velocitySupplier_mps.getAsDouble());
        });
  }

  /** Sets voltage of both motors to 0. */
  public void stop() {
    this.getCurrentCommand().cancel();
    this.startEnd(() -> setVoltage(0), () -> {}).schedule();
  }

  /** Returns the inputs of the elevator mechanism. */
  public ElevatorIOInputs getMotorInputs() {
    return ioInputs;
  }

  private TrapezoidProfile.State calculateNextState(TrapezoidProfile.State goalState) {
    return profile.calculate(
        0.02,
        new TrapezoidProfile.State(ioInputs.mechanismPosition_m, ioInputs.mechanismVelocity_mps),
        goalState);
  }
}
