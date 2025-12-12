package frc.robot.util.logging;

public class LogUtil {
  //   public static void Log(String key, Measure<?> value) {
  //     Logger.recordOutput(
  //         key + toSuffix(value.baseUnit().symbol()), (double) value.baseUnitMagnitude());
  //   }

  //   public static <U extends Unit> void Log(String key, Measure<U> value, U unit) {
  //     Logger.recordOutput(key + toSuffix(unit.symbol()), (double) value.in(unit));
  //   }

  //   public static <U extends Unit> void Log(String key, double value, U unit) {
  //     Logger.recordOutput(key + toSuffix(unit.symbol()), value);
  //   }

  //   public static void Log(String key, double value, String unit) {
  //     Logger.recordOutput(key + toSuffix(unit), value);
  //   }

  //   public static <T extends StructSerializable> void Log(String key, T value) {
  //     Logger.recordOutput(key, value);
  //   }

  public static String toSuffix(String symbol) {
    return "_"
        + symbol
            .replace("u", "µ")
            .replace("*", "·")
            .replace('K', 'k')
            .replace("/", " ̸ ")
            .replace("<?>", "value");
  }
}
