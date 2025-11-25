package frc.robot.util.motionProfiling;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.logging.LogUtil;
import frc.robot.util.motionProfiling.ProfileController.ProfileState;

public class TrapezoidProfileController<
    U extends Unit, V extends PerUnit<U, TimeUnit>, A extends PerUnit<V, TimeUnit>> {

  // Constants
  protected final U pUnit;
  protected final V vUnit;
  protected final A aUnit;

  protected final double minP;
  protected final double maxP;
  protected final double maxV;
  protected final double maxA;

  protected final String setpointLogName;
  protected final String profileStateLogName;

  // Variables
  protected TrapezoidProfile profile;

  protected boolean closedLoop;
  protected Voltage motorVoltage;
  protected ProfileState<U, V> profileState;

  protected State nextState = new State(Double.NaN, Double.NaN);
  protected State lastNextState = new State(Double.NaN, Double.NaN);
  protected State goalState = new State(Double.NaN, Double.NaN);

  public TrapezoidProfileController(
      Measure<U> minP,
      Measure<U> maxP,
      Measure<V> maxV,
      Measure<A> maxA,
      U pUnit,
      V vUnit,
      A aUnit,
      String logName) {
    this.pUnit = pUnit;
    this.vUnit = vUnit;
    this.aUnit = aUnit;
    this.minP = minP.in(pUnit);
    this.maxP = maxP.in(pUnit);
    this.maxV = maxV.in(vUnit);
    this.maxA = maxA.in(aUnit);
    logName = logName.endsWith("/") ? logName : logName + "/";
    this.setpointLogName = logName + "Setpoint/";
    this.profileStateLogName = logName + "ProfileState/";
  }

  public void updateState(State currentState) {
    if (nextState.equals(new State(Double.NaN, Double.NaN))) {
      lastNextState = currentState;
    } else {
      lastNextState = nextState;
    }
    nextState = new State(Double.NaN, Double.NaN);
  }

  public void setVoltage(Voltage voltage) {
    LogUtil.Log(setpointLogName, voltage, Volts);
    LogUtil.Log(setpointLogName, Double.NaN, pUnit);
    LogUtil.Log(setpointLogName, Double.NaN, vUnit);
    closedLoop = false;
    motorVoltage = voltage;
  }

  public void setGoalPosition(Measure<U> position) {
    LogUtil.Log(setpointLogName, Double.NaN, Volts);
    LogUtil.Log(setpointLogName, position, pUnit);
    LogUtil.Log(setpointLogName, Double.NaN, vUnit);

    closedLoop = true;
    profile = new TrapezoidProfile(new Constraints(maxV, maxA));
    goalState = new State(position.in(pUnit), 0);
  }

  public void setGoalVelocity(Measure<V> velocity) {
    LogUtil.Log(setpointLogName, Double.NaN, Volts);
    LogUtil.Log(setpointLogName, Double.NaN, pUnit);
    LogUtil.Log(setpointLogName, velocity, vUnit);

    closedLoop = true;
    profile =
        new TrapezoidProfile(
            new Constraints(velocity.abs(vUnit) < maxV ? velocity.abs(vUnit) : maxV, maxA));
    goalState = new State(velocity.in(vUnit) >= 0 ? maxP : minP, 0);
  }

  public Voltage getVoltage() {
    if (closedLoop) {
      return null;
      // TODO: make this an alert
    }
    return motorVoltage;
  }

  @SuppressWarnings("unchecked")
  public ProfileState<U, V> getNextProfileState() {
    if (!closedLoop) {
      return null;
      // TODO: make this an alert
    }
    nextState = profile.calculate(0.02, lastNextState, goalState);

    profileState.lastNextPosition = (Measure<U>) pUnit.of(lastNextState.position);
    profileState.lastNextVelocity = (Measure<V>) vUnit.of(lastNextState.velocity);
    profileState.nextPosition = (Measure<U>) pUnit.of(nextState.position);
    profileState.nextVelocity = (Measure<V>) vUnit.of(nextState.velocity);

    LogUtil.Log(profileStateLogName + "LastNext", profileState.lastNextPosition, pUnit);
    LogUtil.Log(profileStateLogName + "LastNext", profileState.lastNextVelocity, vUnit);
    LogUtil.Log(profileStateLogName + "Next", profileState.nextPosition, pUnit);
    LogUtil.Log(profileStateLogName + "Next", profileState.nextVelocity, vUnit);

    return profileState;
  }

  // =
  // new TrapezoidProfile(new Constraints(maxVelocity, maxAcceleration));
}
