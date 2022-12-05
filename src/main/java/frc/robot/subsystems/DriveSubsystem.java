// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.DataLogger;
import frc.robot.utils.DataLogger.LogEntry;
import frc.robot.utils.TractionControlController;
import frc.robot.utils.TurnPIDController;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    private WPI_TalonSRX lFrontMotor, rFrontMotor;
    private WPI_TalonSRX lRearMotor, rRearMotor;
    private AHRS navx;

    public Hardware(WPI_TalonSRX lFrontMotor, 
                    WPI_TalonSRX rFrontMotor, 
                    WPI_TalonSRX lRearMotor,
                    WPI_TalonSRX rRearMotor,
                    AHRS navx) {
      this.lFrontMotor = lFrontMotor;
      this.rFrontMotor = rFrontMotor;
      this.lRearMotor = lRearMotor;
      this.rRearMotor = rRearMotor;
      this.navx = navx;
    }

  }

  private WPI_TalonSRX m_lFrontMotor, m_rFrontMotor;
  private WPI_TalonSRX m_lRearMotor, m_rRearMotor;

  private TractionControlController m_tractionControlController;
  private AHRS m_navx;
  private TurnPIDController m_turnPIDController;

  private double m_deadband;

  private final double MOTOR_DEADBAND = 0.07;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param turnScalar Scalar for turn input (degrees)
   * @param deadband Deadband for controller input [+0.001, +0.1]
   * @param lookAhead Turn PID lookahead, in number of loops
   * @param metersPerTick Meters traveled per encoder tick (meters)
   * @param maxLinearSpeed Maximum linear speed of the robot (m/s)
   * @param tractionControlCurve Spline function characterising traction of the robot
   * @param throttleInputCurve Spline function characterising throttle input
   * @param turnInputCurve Spline function characterising turn input
   */
  public DriveSubsystem(
      Hardware drivetrainHardware, 
      double kP,
      double kD,
      double turnScalar,
      double deadband,
      double lookAhead,
      double metersPerTick,
      double maxLinearSpeed,
      PolynomialSplineFunction tractionControlCurve, 
      PolynomialSplineFunction throttleInputCurve,
      PolynomialSplineFunction turnInputCurve
    ) {
    this.m_lFrontMotor = drivetrainHardware.lFrontMotor;
    this.m_rFrontMotor = drivetrainHardware.rFrontMotor;
    this.m_lRearMotor = drivetrainHardware.lRearMotor;
    this.m_rRearMotor = drivetrainHardware.rRearMotor;
    this.m_navx = drivetrainHardware.navx;

    this.m_deadband = deadband;

    this.m_lFrontMotor.setNeutralMode(NeutralMode.Brake);
    this.m_rFrontMotor.setNeutralMode(NeutralMode.Brake);
    this.m_lRearMotor.setNeutralMode(NeutralMode.Brake);
    this.m_rRearMotor.setNeutralMode(NeutralMode.Brake);

    this.m_lFrontMotor.configNeutralDeadband(MOTOR_DEADBAND);
    this.m_rFrontMotor.configNeutralDeadband(MOTOR_DEADBAND);
    this.m_lRearMotor.configNeutralDeadband(MOTOR_DEADBAND);
    this.m_rRearMotor.configNeutralDeadband(MOTOR_DEADBAND);

    m_rFrontMotor.setInverted(true);
    m_rRearMotor.setInverted(true);

    m_lRearMotor.follow(m_lFrontMotor);
    m_rRearMotor.follow(m_rFrontMotor);

    m_turnPIDController = new TurnPIDController(kP, kD, turnScalar, deadband, lookAhead, turnInputCurve);
    m_tractionControlController = new TractionControlController(deadband, maxLinearSpeed, tractionControlCurve, throttleInputCurve);

    m_navx.calibrate();
    resetAngle();
    m_turnPIDController.setSetpoint(0.0);
    m_turnPIDController.setTolerance(0.125);

    addLogEntries();
  }

  private void resetAngle() {
    m_navx.reset();
  }

  private void addLogEntries() {
    DataLogger logger = DataLogger.getInstance();
    logger.addEntry(new LogEntry(() -> { return m_navx.getAngle(); }, "Angle"));
    logger.addEntry(new LogEntry(() -> { return m_lFrontMotor.getSupplyCurrent(); }, "Front Left Motor Current"));
    logger.addEntry(new LogEntry(() -> { return m_rFrontMotor.getSupplyCurrent(); }, "Front Right Motor Current"));
    logger.addEntry(new LogEntry(() -> { return m_lRearMotor.getSupplyCurrent(); }, "Rear Left Motor Current"));
    logger.addEntry(new LogEntry(() -> { return m_rRearMotor.getSupplyCurrent(); }, "Rear Right Motor Current"));
  }

  /**
   * Initialize hardware devices for drive subsystem
   * @return hardware object containing all necessary devices for this subsystem
   */
  public static Hardware initializeHardware() {
    Hardware drivetrainHardware = new Hardware(new WPI_TalonSRX(Constants.FRONT_LEFT_MOTOR_PORT),
                                               new WPI_TalonSRX(Constants.FRONT_RIGHT_MOTOR_PORT),
                                               new WPI_TalonSRX(Constants.REAR_LEFT_MOTOR_PORT),  
                                               new WPI_TalonSRX(Constants.REAR_RIGHT_MOTOR_PORT),
                                               new AHRS(SPI.Port.kMXP));
    return drivetrainHardware;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * Call this repeatedly to drive during teleop
   * @param ySpeed Desired speed [-1.0, +1.0]
   * @param turn Turn input [-1.0, +1.0]
   */
  public void teleop(double speed, double turn) {
    speed = MathUtil.applyDeadband(speed, m_deadband);
    turn = MathUtil.applyDeadband(turn, m_deadband);

    m_lFrontMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, -turn);
    m_rFrontMotor.set(ControlMode.PercentOutput, speed, DemandType.ArbitraryFeedForward, +turn);
  }

  /**
   * Call this repeatedly to drive using PID during teleop
   * @param speedRequest Desired speed [-1.0, +1.0]
   * @param turnRequest Turn input [-1.0, +1.0]
   */
  public void teleopPID(double speedRequest, double turnRequest) {
    // Calculate next speed output
    double speedOutput = m_tractionControlController.calculate(getInertialVelocityX(), speedRequest);

    // Calculate next turn output
    double turnOutput = m_turnPIDController.calculate(getAngle(), getTurnRate(), turnRequest);

    // Run motors with appropriate values
    m_lFrontMotor.set(ControlMode.PercentOutput, speedOutput, DemandType.ArbitraryFeedForward, -turnOutput);
    m_rFrontMotor.set(ControlMode.PercentOutput, speedOutput, DemandType.ArbitraryFeedForward, +turnOutput);
  }

  /**
   * Inertial velocity in X axis 
   * @return velocity in m/s
   */
  public double getInertialVelocityX() {
    return m_navx.getVelocityX();
  }

  /**
   * Inertial velocity in Y axis
   * @return velocity in m/s
   */
  public double getInertialVelocityY() {
    return m_navx.getVelocityY();
  }

  /**
   * Angle of robot
   * @return angle in degrees
   */
  public double getAngle() {
    return m_navx.getAngle();
  }

  /**
   * Turn rate of robot
   * @return turn rate in degrees per second
   */
  public double getTurnRate() {
    return m_navx.getRate();
  }

  @Override
  public void close() {
    m_lFrontMotor.close();
    m_rFrontMotor.close();
    m_lRearMotor.close();
    m_rRearMotor.close();
  }
}