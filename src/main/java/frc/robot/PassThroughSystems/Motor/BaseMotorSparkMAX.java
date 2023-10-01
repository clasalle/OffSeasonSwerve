package frc.robot.PassThroughSystems.Motor;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;
import frc.robot.Constants.PIDConstantsForSwerveModules.MAXAngle;
import frc.robot.Constants.SwerveChassis.SwerveModuleConstants;
import frc.robot.Constants.SwerveChassis.SparkMAXSwerveConfiguration;

/*
 * This is a specific base motor implementation for the motors connected to TalonSRX
 */
public class BaseMotorSparkMAX implements BaseMotorInterface {
  private CANSparkMax motorSparkMax;
  private SparkMaxPIDController mSparkMaxPIDController;

  public BaseMotorSparkMAX(int CANID) {
      System.out.println("**** Activating SparkMAX NEO CANID:" + CANID);

      motorSparkMax = new CANSparkMax(CANID, MotorType.kBrushless);
  }

  public void configureDriveMotor(Constants.SwerveChassis.SwerveModuleConstants c) {
      //TODO: Add logic to configureDriveMotor

      motorBrakeMode();
  }

  public void configureAngleMotor(SwerveModuleConstants c) {
      //TODO: Add logic to configureAngleMotor

      motorBrakeMode();
  }

  public double getDriveEncoderPosition() {
      return motorSparkMax.getEncoder().getPosition();
  }

  public double getAngleEncoderPosition() {
      return motorSparkMax.getEncoder().getPosition();
  }

  public double getDriveEncoderVelocity() {
      return motorSparkMax.getEncoder().getVelocity();
  }

  public double getAngleEncoderVelocity() {
      return motorSparkMax.getEncoder().getVelocity();
  }

  public double getDriveEncoderPositionSI() {
     
      return getDriveEncoderPosition()*Constants.SwerveChassis.SparkMAXSwerveConfiguration.metersPerTick;
  }

  public double getAngleEncoderPositionSI() {
    
      return getAngleEncoderPosition()*Constants.SwerveChassis.SparkMAXSwerveConfiguration.degreePerTick;
  }

  public double getDriveEncoderVelocitySI() {
 
      return getDriveEncoderVelocity()*Constants.SwerveChassis.SparkMAXSwerveConfiguration.metersPerTick;
  }

  public double getAngleEncoderVelocitySI() {
     
      return getAngleEncoderVelocity()*Constants.SwerveChassis.SparkMAXSwerveConfiguration.degreePerTick;
  }

  public void setAngleMotorChassisAngleSI(double angle) {
     motorSparkMax.getPIDController().setReference(angle, ControlType.kPosition);
  }

  public void testMotorApplyPower(double power) {
     motorSparkMax.set(power);
  }

  public void applyPower(double power) {
     motorSparkMax.set(power);
  }

     private double degreesToTicks(double degrees) {
        return degrees / SparkMAXSwerveConfiguration.degreePerTick;
    }

    

    private void configureMotionMagicAngle(Constants.SwerveChassis.SwerveModuleConstants c) {

        // Disable motor safety so we can use hardware PID
       // motorSparkMax.setSafetyEnabled(false);

        //motorSparkMax.configNeutralDeadband(MAXAngle.NeutralDeadband, MAXAngle.timeoutMs);

        motorSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXAngle.periodMs);
        motorSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, MAXAngle.periodMs);

        motorSparkMax.setSecondaryCurrentLimit(+1.0, MAXAngle.timeoutMs);
        motorSparkMax.setSecondaryCurrentLimit(+1.0, MAXAngle.timeoutMs);
        mSparkMaxPIDController.setSmartMotionMinOutputVelocity(0, MAXAngle.timeoutMs);

        /* FPID Gains */
        mSparkMaxPIDController.setP(MAXAngle.kP, MAXAngle.SLOT_0);
        mSparkMaxPIDController.setI(MAXAngle.kI, MAXAngle.SLOT_0);
        mSparkMaxPIDController.setD(MAXAngle.kD, MAXAngle.SLOT_0);
        mSparkMaxPIDController.setFF(MAXAngle.kF, MAXAngle.SLOT_0);

        mSparkMaxPIDController.setIZone(MAXAngle.Izone, MAXAngle.SLOT_0);
        mSparkMaxPIDController.setOutputRange(0, MAXAngle.PeakOutput, MAXAngle.SLOT_0);
        mSparkMaxPIDController.setSmartMotionAllowedClosedLoopError(MAXAngle.DefaultAcceptableError, MAXAngle.SLOT_0);
      
        //mSparkMaxPIDController.setClosedLoopPeriod(MAXAngle.closedLoopPeriod);

        mSparkMaxPIDController.setSmartMotionMaxAccel(MAXAngle.Acceleration,MAXAngle.timeoutMs);
        mSparkMaxPIDController.setSmartMotionMaxVelocity(MAXAngle.CruiseVelocity,MAXAngle.timeoutMs);
        mSparkMaxPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, MAXAngle.Smoothing);
    }

    // Current limiter configuration for the angle motor
    private void configureCurrentLimiterAngle() {
        motorSparkMax.setSecondaryCurrentLimit(SparkMAXSwerveConfiguration.anglePeakCurrentLimit, SparkMAXSwerveConfiguration.configureTimeoutMs);
		motorSparkMax.configPeakCurrentDuration(SparkMAXSwerveConfiguration.anglePeakCurrentDuration, SparkMAXSwerveConfiguration.configureTimeoutMs);
		motorSparkMax.configContinuousCurrentLimit(SparkMAXSwerveConfiguration.angleContinuousCurrentLimit, SparkMAXSwerveConfiguration.configureTimeoutMs);
		motorSparkMax.enableCurrentLimit(SparkMAXSwerveConfiguration.angleEnableCurrentLimit); // Honor initial setting

    }

    // Current limiter configuration for the drive motor
    private void configureCurrentLimiterDrive() {
        motorSparkMax.configPeakCurrentLimit(SparkMAXSwerveConfiguration.drivePeakCurrentLimit, SparkMAXSwerveConfiguration.configureTimeoutMs);
		motorSparkMax.configPeakCurrentDuration(SparkMAXSwerveConfiguration.drivePeakCurrentDuration, SparkMAXSwerveConfiguration.configureTimeoutMs);
		motorSparkMax.configContinuousCurrentLimit(SparkMAXSwerveConfiguration.driveContinuousCurrentLimit, SparkMAXSwerveConfiguration.configureTimeoutMs);
		motorSparkMax.enableCurrentLimit(SparkMAXSwerveConfiguration.driveEnableCurrentLimit); // Honor initial setting

    }

    public void initQuadrature() { // Set absolute encoders
        int pulseWidth = motorSparkMax.getSensorCollection().getPulseWidthPosition();

        if (SparkMAXSwerveConfiguration.kDiscontinuityPresent) {

            /* Calculate the center */
            int newCenter;
            newCenter = (SparkMAXSwerveConfiguration.kBookEnd_0 + SparkMAXSwerveConfiguration.kBookEnd_1) / 2;
            newCenter &= 0xFFF;

            /**
             * Apply the offset so the discontinuity is in the unused portion of
             * the sensor
             */
            pulseWidth -= newCenter;
        }
    }

    /**
     * The CTR Mag encoders we use to track wheel angle can be used in both absolute and relative modes
     * at the same time. The Hardware PID on the SparkMAX, however, is easier to use with relative encoders.
     * So, we read absolute encoders at the start, and set relative encoders so their 0 corresponds to the
     * wheels being lined up and facing forward (0 degree from the forward robot direction).
     * We have not found significant drift/discrepancy between absolute and relative encoder increments, so
     * we do not currently recalibrate relative encoders again during the game.
     * Note that the wheels do not need to be set "forward" at the beginning of the game. The absolute encoder
     * will set the right angle-related value to the relative encoder, since absolute encoders are not set to 0 after
     * power cycle. The drive routines will then change the wheel positions as needed.
    */
    public void setEncoderforWheelCalibration(SwerveModuleConstants c) {
        double difference = getDriveAbsEncoder() - c.getAngleOffset()*4096.0/360.0;
        double encoderSetting = 0.0;

        if (difference < 0) {
            difference += SparkMAXSwerveConfiguration.clicksSRXPerFullRotation;
        }

        if (difference <= SparkMAXSwerveConfiguration.clicksSRXPerFullRotation / 2) {
            encoderSetting = difference;

        } else {
            encoderSetting = difference - SparkMAXSwerveConfiguration.clicksSRXPerFullRotation;
        }

        motorSparkMax.setSelectedSensorPosition(encoderSetting);

        System.out.println("Set encoder for motor " + c.getAngleMotorID() + " to " + encoderSetting);

    }

    private void motorBrakeMode() {
        motorSparkMax.setIdleMode(IdleMode.kBrake);
    }

}
