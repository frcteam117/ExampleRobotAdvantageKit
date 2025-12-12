package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.util.logging.InputsStruct;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ElevatorIO {
  public class ElevatorIOInputs implements LoggableInputs, StructSerializable {
    /** Elevator's leading motor's internal encoder position. */
    public Angle rotorAngle = Rotations.of(0);
    /** Internal encoder velocity of the elevator's leading motor. */
    public AngularVelocity rotorVelocity = RotationsPerSecond.of(0);

    /**
     * Scaled height of the elevator based on the internal encoder position of its leading motor.
     */
    public Distance mechanismHeight = Meters.of(0);
    /**
     * Scaled velocity of the elevator based on the internal encoder velocity of its leading motor.
     */
    public LinearVelocity mechanismVelocity = MetersPerSecond.of(0);

    /** Voltage applied to the stator of the elevator's leading motor. */
    public Voltage motorVoltage = Volts.of(0);

    /** Current applied to the stator of the elevator's leading motor. */
    public Current statorCurrent = Amps.of(0);
    /**
     * Current supplied to the motor controller of the elevator's leading motor. Approximately half
     * of the total supply current of the elevator as a whole
     */
    public Current supplyCurrent = Amps.of(0);

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
      rotorAngle = inputs.rotorAngle;
      rotorVelocity = inputs.rotorVelocity;
      mechanismHeight = inputs.mechanismHeight;
      mechanismVelocity = inputs.mechanismVelocity;
      motorVoltage = inputs.motorVoltage;
      statorCurrent = inputs.statorCurrent;
      supplyCurrent = inputs.supplyCurrent;
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
   * @param voltage the voltage applied to the motors
   */
  default void setVoltage(Voltage voltage) {}

  /**
   * Sets the voltage based on a feedforard value added to a position pid.
   *
   * @param nextState the target position and velocity of the elevator in the next timestep
   */
  default void setNextState(Distance nextHeight, LinearVelocity nextVelocity) {}
}
