// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import java.util.function.Supplier;
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
  public void setVoltage(Voltage voltage) {
    io.setVoltage(voltage);
  }

  /** Sets the elevator to the supplied voltage. */
  public Command setVoltageCommandFactory(Supplier<Voltage> voltageSupplier) {
    return this.runOnce(
        () -> {
          setVoltage(voltageSupplier.get());
        });
  }

  /** Runs the elevator at the supplied voltage. */
  public Command runVoltageCommandFactory(Supplier<Voltage> voltageSupplier) {
    return this.run(
        () -> {
          setVoltage(voltageSupplier.get());
        });
  }

  // Position methods

  /** Runs the elevator to and holds at the supplied height. */
  public void setGoalPosition(Distance height) {
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity_mps, maxAcceleration_mps2));
    TrapezoidProfile.State nextState =
        calculateNextState(new TrapezoidProfile.State(height.in(Meters), 0));
    io.setNextState(Meters.of(nextState.position), MetersPerSecond.of(nextState.velocity));
  }

  /** Runs the elevator to and holds at the supplied height. */
  public Command setGoalPositionCommandFactory(Supplier<Distance> heightSupplier) {
    return this.runOnce(
        () -> {
          setGoalPosition(heightSupplier.get());
        });
  }

  /** Runs the elevator to and holds at the supplied height. */
  public Command runGoalPositionCommandFactory(Supplier<Distance> heightSupplier) {
    return this.run(
        () -> {
          setGoalPosition(heightSupplier.get());
        });
  }

  // Velocity methods

  /** Runs the elevator to and holds at the supplied velocity. */
  public void setGoalVelocity(LinearVelocity velocity) {
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity_mps, maxAcceleration_mps2));
    TrapezoidProfile.State nextState =
        calculateNextState(
            new TrapezoidProfile.State(
                velocity.in(MetersPerSecond) >= 0 ? maxPosition_m : minPosition_m,
                velocity.in(MetersPerSecond)));
    io.setNextState(Meters.of(nextState.position), MetersPerSecond.of(nextState.velocity));
  }

  /** Runs the elevator to and holds at the supplied velocity. */
  public Command setGoalVelocityCommandFactory(Supplier<LinearVelocity> velocitySupplier) {
    return this.runOnce(
        () -> {
          setGoalVelocity(velocitySupplier.get());
        });
  }

  /** Runs the elevator to and holds at the supplied velocity. */
  public Command runGoalVelocityCommandFactory(Supplier<LinearVelocity> velocitySupplier) {
    return this.run(
        () -> {
          setGoalVelocity(velocitySupplier.get());
        });
  }

  /** Sets voltage of both motors to 0. */
  public void stop() {
    this.getCurrentCommand().cancel();
    this.startEnd(() -> setVoltage(Volts.of(0)), () -> {}).schedule();
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
