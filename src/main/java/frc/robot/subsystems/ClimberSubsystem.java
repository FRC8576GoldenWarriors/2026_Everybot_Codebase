package frc.robot.subsystems;

import static frc.robot.Constants.ClimbConstatns.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.RevUtil;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax climberMotor;
  private final Alert climbAlert;

  /** Creates a new CANBallSubsystem. */
  public ClimberSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushed);
    climbAlert = new Alert("Climb/", "Climb Motor (8)", AlertType.kError);

    // create the configuration for the climb moter, set a current limit and apply
    // the config to the controller
    SparkMaxConfig climbConfig = new SparkMaxConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climberMotor.configure(
        climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the percentage of the climber
  public void setClimber(double power) {
    climberMotor.set(power);
  }

  // A method to stop the climber
  public void stop() {
    climberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Climb/Climber Velocity", climberMotor.get());
    var climbStatus = RevUtil.checkSparkMaxState(climberMotor.getLastError());
    climbAlert.set(climbStatus.getFirst());
    climbAlert.setText(climbStatus.getSecond());
  }
}
