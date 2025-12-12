package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.util.logging.InputsStruct;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ElevatorIO {
  public class ElevatorIOInputs implements LoggableInputs, StructSerializable {
    /** Elevator's leading motor's internal encoder position. */
    public double rotorAngle_rot = 0;
    /** Internal encoder velocity of the elevator's leading motor. */
    public double rotorVelocity_rotps = 0;

    /**
     * Scaled height of the elevator based on the internal encoder position of its leading motor.
     */
    public double mechanismPosition_m = 0;
    /**
     * Scaled velocity of the elevator based on the internal encoder velocity of its leading motor.
     */
    public double mechanismVelocity_mps = 0;

    /** Voltage applied to the stator of the elevator's leading motor. */
    public double motorVoltage_V = 0;

    /** Current applied to the stator of the elevator's leading motor. */
    public double statorCurrent_A = 0;
    /**
     * Current supplied to the motor controller of the elevator's leading motor. Approximately half
     * of the total supply current of the elevator as a whole
     */
    public double supplyCurrent_A = 0;

    public static final Struct<ElevatorIOInputs> struct =
        new InputsStruct<>(
            ElevatorIOInputs.class,
            Rotations,
            RotationsPerSecond,
            Meters,
            MetersPerSecond,
            Volts,
            Amps,
            Amps);

    @Override
    public void toLog(LogTable table) {
      table.put("Inputs", struct, this);
    }

    @Override
    public void fromLog(LogTable table) {
      ElevatorIOInputs inputs = table.get("Inputs", struct, this);
      rotorAngle_rot = inputs.rotorAngle_rot;
      rotorVelocity_rotps = inputs.rotorVelocity_rotps;
      mechanismPosition_m = inputs.mechanismPosition_m;
      mechanismVelocity_mps = inputs.mechanismVelocity_mps;
      motorVoltage_V = inputs.motorVoltage_V;
      statorCurrent_A = inputs.statorCurrent_A;
      supplyCurrent_A = inputs.supplyCurrent_A;
    }
  }

  /**
   * Mutates the provided {@code ElevatorIOInputs} into the current inputs of the elevator. Also
   * runs the update method for simulation.
   *
   * @param inputs the {@code ElevatorIOInputs} to be mutated
   */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the voltage applied to the elevator's motors in an open loop
   *
   * @param voltage_V the voltage applied to the motors
   */
  default void setVoltage(double voltage_V) {}

  /**
   * Sets the voltage based on a feedforard value added to a position pid.
   *
   * @param nextHeight_m the target position of the elevator in the next timestep
   * @param nextVelocity_mps the target velocity of the elevator in the next timestep
   */
  default void setNextState(double nextHeight_m, double nextVelocity_mps) {}
}
