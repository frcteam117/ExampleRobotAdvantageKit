package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.motionProfiling.ProfileController.ProfileState;

public interface ElevatorIO {
  public class ElevatorIOState {
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
  }

  /**
   * Mutates the provided {@code ElevatorIOState} into the current state of the elevator. Also runs
   * the update method for simulation.
   *
   * @param state the {@code ElevatorIOState} to be mutated
   */
  default void updateState(ElevatorIOState state) {}

  /**
   * Sets the voltage applied to the elevator's motors in an open loop
   *
   * @param voltage the voltage applied to the motors
   */
  default void setVoltage(Voltage voltage) {}

  default void setNextState(ProfileState<DistanceUnit, LinearVelocityUnit> nextState) {}
  /**
   * Runs the elevator in a closed loop to and holds at the height provided
   *
   * @param height the height to run the elevator to
   */
  default void setGoalPosition(Distance height) {}

  /**
   * Runs the elevator in a closed loop to and hold at the velocity provided
   *
   * @param velocity the velocity to run the elevator to
   */
  default void setGoalVelocity(LinearVelocity velocity) {}
}
