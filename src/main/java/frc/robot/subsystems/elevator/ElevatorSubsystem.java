// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOState;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.logging.MeasureStruct;
import frc.robot.util.motionProfiling.TrapezoidProfileController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  /** Interface to control the elevator's motors */
  private final ElevatorIO io;

  /** State of the elevator's motors */
  private final ElevatorIOState ioState = new ElevatorIOState();

  private final TrapezoidProfileController<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit>
      profileController =
          new TrapezoidProfileController<>(
              minPosition,
              maxPosition,
              maxVelocity,
              maxAcceleration,
              Meters,
              MetersPerSecond,
              MetersPerSecondPerSecond,
              "Elevator");

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateState(ioState);
    logState();
    profileController.updateState(
        new State(
            ioState.mechanismHeight.in(Meters), ioState.mechanismVelocity.in(MetersPerSecond)));
  }

  // Voltage methods

  /** Sets the elevator to the supplied voltage. */
  public void setVoltage(Voltage voltage) {
    profileController.setVoltage(voltage);
    io.setVoltage(profileController.getVoltage());
  }

  /** Sets the elevator to the supplied voltage. */
  public Command setVoltageCommandFactory(Supplier<Voltage> voltageSupplier) {
    return runOnce(
        () -> {
          setVoltage(voltageSupplier.get());
        });
  }

  /** Runs the elevator at the supplied voltage. */
  public Command runVoltageCommandFactory(Supplier<Voltage> voltageSupplier) {
    return run(
        () -> {
          setVoltage(voltageSupplier.get());
        });
  }

  // Position methods

  /** Runs the elevator to and holds at the supplied height. */
  public void setGoalPosition(Distance height) {
    profileController.setGoalPosition(height);
    io.setNextState(profileController.getNextProfileState());
  }

  /** Runs the elevator to and holds at the supplied height. */
  public Command setGoalPositionCommandFactory(Supplier<Distance> heightSupplier) {
    return runOnce(
        () -> {
          setGoalPosition(heightSupplier.get());
        });
  }

  /** Runs the elevator to and holds at the supplied height. */
  public Command runGoalPositionCommandFactory(Supplier<Distance> heightSupplier) {
    return run(
        () -> {
          setGoalPosition(heightSupplier.get());
        });
  }

  // Velocity methods

  /** Runs the elevator to and holds at the supplied velocity. */
  public void setGoalVelocity(LinearVelocity velocity) {
    profileController.setGoalVelocity(velocity);
    io.setNextState(profileController.getNextProfileState());
    ;
  }

  /** Runs the elevator to and holds at the supplied velocity. */
  public Command setGoalVelocityCommandFactory(Supplier<LinearVelocity> velocitySupplier) {
    return runOnce(
        () -> {
          setGoalVelocity(velocitySupplier.get());
        });
  }

  /** Runs the elevator to and holds at the supplied velocity. */
  public Command runGoalVelocityCommandFactory(Supplier<LinearVelocity> velocitySupplier) {
    return run(
        () -> {
          setGoalVelocity(velocitySupplier.get());
        });
  }

  /** Sets voltage of both motors to 0. */
  public void stop() {
    setVoltageCommandFactory(() -> Volts.of(0.0)).schedule();
    ;
  }

  /** Returns the motor state of the elevator. */
  public ElevatorIOState getMotorState() {
    return ioState;
  }

  private void logState() {
    Logger.recordOutput(
        name + "/distancetest", new MeasureStruct<DistanceUnit>(Meters), Feet.of(30));
    // LoggedDoubleList ldl = new LoggedDoubleList(name + "/Setpoint938", "_m", "_m ̸ s", "_V");
    // ldl.setValues(2.4, 4.9, 5.0);
    // Logger.recordOutput(
    //     name + "/Setpoint22",
    //     Setpoint.struct,
    //     new Setpoint(Meters.of(Double.NaN), FeetPerSecond.of(2), Volts.of(4)));
    // Logger.recordOutput(name + "/Setpoint63", ldl.struct, ldl.values);
    LogUtil.Log("/rotorAngle", ioState.rotorAngle);
    LogUtil.Log("/rotorVelocity", ioState.rotorVelocity);
    LogUtil.Log("/mechanismHeight", ioState.mechanismHeight);
    LogUtil.Log("/mechanismVelocity", ioState.mechanismVelocity);
    LogUtil.Log("/motorVoltage", ioState.motorVoltage);
    LogUtil.Log("/statorCurrent", ioState.statorCurrent);
    LogUtil.Log("/supplyCurrent", ioState.supplyCurrent);
  }
}
