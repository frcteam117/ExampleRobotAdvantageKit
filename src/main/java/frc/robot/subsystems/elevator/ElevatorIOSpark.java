// Copyright 2021-2025 FRC 6328
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

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOSpark implements ElevatorIO {
    // Sparkmax objects
    private SparkMax leadingSpark = new SparkMax(LeaderCanId, MotorType.kBrushless);
    private SparkMax followingSpark = new SparkMax(FollowerCanId, MotorType.kBrushless);
    private RelativeEncoder encoder = leadingSpark.getEncoder();

    // Motion profiling
    private ElevatorFeedforward feedforward =
            new ElevatorFeedforward(realS, realG, realV, realA, 0.02);
    private PIDController pid = new PIDController(realP, realI, realD);

    public ElevatorIOSpark() {
        leadingSpark.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        followingSpark.configure(
                motorConfig.follow(leadingSpark, true),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    // Variables
    private double lastNextPositionMeters;

    @Override
    public void updateState(ElevatorIOState ioState) {
        ioState.rotorAngle = Rotations.of(encoder.getPosition());
        ioState.rotorVelocity = RotationsPerSecond.of(encoder.getVelocity());
        ioState.mechanismHeight = Meters.of(encoder.getPosition() / rotationsperMeter);
        ioState.mechanismVelocity = MetersPerSecond.of(encoder.getVelocity() / rotationsperMeter);

        ioState.motorVoltage =
                Volts.of(leadingSpark.getBusVoltage() * leadingSpark.getAppliedOutput());
        ioState.statorCurrent = Amps.of(leadingSpark.getOutputCurrent());
        ioState.supplyCurrent = ioState.statorCurrent.times(ioState.motorVoltage.in(Volts) / 12.0);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        leadingSpark.setVoltage(voltage.in(Volts));
        lastNextPositionMeters = encoder.getPosition();
    }

    @Override
    public void setNextState(Distance nextHeight, LinearVelocity nextVelocity) {
        leadingSpark.setVoltage(
                feedforward.calculateWithVelocities(
                                encoder.getVelocity(), nextVelocity.in(MetersPerSecond))
                        + pid.calculate(encoder.getPosition(), lastNextPositionMeters));
        lastNextPositionMeters = nextHeight.in(Meters);
    }
}
