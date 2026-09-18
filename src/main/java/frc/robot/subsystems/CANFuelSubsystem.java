// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.*;

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

public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax LeftIntakeLauncher;
  private final SparkMax RightIntakeLauncher;
  private final SparkMax Indexer;

  private final Alert LeftIntakeLauncherAlert;
  private final Alert RightIntakeLauncherAlert;
  private final Alert IndexerAlert;

  private final String LeftIntakeLauncherAlertBaseText = "Left Intake Launcher Motor (5): ";
  private final String RightIntakeLauncherAlertBaseText = "Right Intake Launcher Motor (6): ";
  private final String IndexerAlertBaseText = "Indexer Motor (6): ";

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {

    LeftIntakeLauncherAlert = new Alert("Fuel/", LeftIntakeLauncherAlertBaseText, AlertType.kError);
    RightIntakeLauncherAlert =
        new Alert("Fuel/", RightIntakeLauncherAlertBaseText, AlertType.kError);
    IndexerAlert = new Alert("Fuel/", IndexerAlertBaseText, AlertType.kError);

    // create brushed motors for each of the motors on the launcher mechanism
    LeftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    RightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig rightIntakeLauncher = new SparkMaxConfig();
    SparkMaxConfig indexerConfig = new SparkMaxConfig();

    Indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
    feederConfig.inverted(INDEXER_INVERTED);
    Indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();

    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherConfig.voltageCompensation(12);
    launcherConfig.idleMode(IdleMode.kCoast);
    RightIntakeLauncher.configure(
        launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    launcherConfig.inverted(true);
    LeftIntakeLauncher.configure(
        launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // put default values for various fuel operations onto the dashboard
    // all commands using this subsystem pull values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    Logger.recordOutput("INDEXER_INTAKING_PERCENT", INDEXER_INTAKING_PERCENT);
    Logger.recordOutput("INTAKE_INTAKING_PERCENT", INTAKE_INTAKING_PERCENT);
    Logger.recordOutput("INDEXER_LAUNCHING_PERCENT", INDEXER_LAUNCHING_PERCENT);
    Logger.recordOutput("LAUNCHING_LAUNCHER_PERCENT", LAUNCHING_LAUNCHER_PERCENT);
    // Logger.recordOutput("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the voltage of the intake roller
  public void setIntakeLauncherRoller(double power) {
    LeftIntakeLauncher.set(power);
    RightIntakeLauncher.set(power); // positive for shooting
  }

  // A method to set the voltage of the intake roller
  public void setFeederRoller(double power) {
    Indexer.set(power); // positive for shooting
  }

  // A method to stop the rollers
  public void stop() {
    Indexer.set(0);
    LeftIntakeLauncher.set(0);
    RightIntakeLauncher.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Fuel/Left Intake Launcher Velocity", LeftIntakeLauncher.get());
    Logger.recordOutput("Fuel/Right Intake Launcher Velocity", RightIntakeLauncher.get());
    Logger.recordOutput("Fuel/Indexer Velocity", Indexer.get());

    var leftIntakeLauncherStatus = RevUtil.checkSparkMaxState(LeftIntakeLauncher.getLastError());
    var rightIntakeLauncherStatus = RevUtil.checkSparkMaxState(RightIntakeLauncher.getLastError());
    var indexerStatus = RevUtil.checkSparkMaxState(Indexer.getLastError());

    LeftIntakeLauncherAlert.set(leftIntakeLauncherStatus.getFirst());
    RightIntakeLauncherAlert.set(rightIntakeLauncherStatus.getFirst());
    IndexerAlert.set(indexerStatus.getFirst());

    LeftIntakeLauncherAlert.setText(
        LeftIntakeLauncherAlertBaseText + leftIntakeLauncherStatus.getSecond());
    RightIntakeLauncherAlert.setText(
        RightIntakeLauncherAlertBaseText + rightIntakeLauncherStatus.getSecond());
    IndexerAlert.setText(IndexerAlertBaseText + indexerStatus.getSecond());
  }
}
