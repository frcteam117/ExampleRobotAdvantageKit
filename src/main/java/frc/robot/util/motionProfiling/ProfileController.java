package frc.robot.util.motionProfiling;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Unit;

public abstract class ProfileController {
    public static class ProfileState<U extends Unit, V extends PerUnit<U, TimeUnit>> {
        public Measure<U> nextPosition;
        public Measure<V> nextVelocity;
        public Measure<U> lastNextPosition;
        public Measure<V> lastNextVelocity;
    }
}
