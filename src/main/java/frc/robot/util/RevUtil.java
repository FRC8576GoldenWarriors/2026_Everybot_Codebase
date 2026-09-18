package frc.robot.util;

import com.revrobotics.REVLibError;
import edu.wpi.first.math.Pair;

public class RevUtil {
  public static Pair<Boolean, String> checkSparkMaxState(REVLibError status) {
    if (status == null || status == REVLibError.kOk) {
      return new Pair<>(false, "OK");
    }

    boolean hasError =
        switch (status) {
          case kCANDisconnected,
              kHALError,
              kCantFindFirmware,
              kFirmwareTooOld,
              kFirmwareTooNew,
              kParamInvalidID,
              kParamMismatchType,
              kParamAccessMode,
              kParamInvalid,
              kError,
              kTimeout ->
              true;
          default -> false;
        };

    String message = hasError ? "SPARK MAX Error: " + status.name() : "OK";

    return new Pair<>(hasError, message);
  }
}
