package swervelib;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Helper class to easily transform Controller inputs into workable Chassis speeds. <br /> Inspired by SciBorgs.
 * https://github.com/SciBorgs/Crescendo-2024/blob/main/src/main/java/org/sciborgs1155/lib/InputStream.java
 * <p>
 * Intended to easily create an interface that generates {@link ChassisSpeeds} from
 * {@link edu.wpi.first.wpilibj.XboxController}
 */
public class SwerveInputStream implements Supplier<ChassisSpeeds>
{

  /**
   * Translation suppliers.
   */
  private final DoubleSupplier            controllerTranslationX;
  /**
   * Translational supplier.
   */
  private final DoubleSupplier            controllerTranslationY;
  /**
   * {@link SwerveDrive} object for transformations.
   */
  private final SwerveDrive               swerveDrive;
  /**
   * Rotation supplier as angular velocity.
   */
  private       Optional<DoubleSupplier>  controllerTheta        = Optional.empty();
  /**
   * Controller supplier as heading.
   */
  private       Optional<DoubleSupplier>  controllerHeadingX     = Optional.empty();
  /**
   * Controller supplier as heading.
   */
  private       Optional<DoubleSupplier>  controllerHeadingY     = Optional.empty();
  /**
   * Axis deadband for the controller.
   */
  private       Optional<Double>          axisDeadband           = Optional.empty();
  /**
   * Translational axis scalar value, should be between (0, 1].
   */
  private       Optional<Double>          translationScale       = Optional.empty();
  /**
   * Angular velocity axis scalar value, should be between (0, 1]
   */
  private       Optional<Double>          angularVelocityScale   = Optional.empty();
  /**
   * Target to aim at.
   */
  private       Optional<Pose2d>          aimTarget              = Optional.empty();
  /**
   * Current years AprilTag field.
   */
  private final AprilTagFieldLayout       field                  = AprilTagFieldLayout
      .loadField(AprilTagFields.k2024Crescendo);
  /**
   * Output {@link ChassisSpeeds} based on heading while this is True.
   */
  private       Optional<BooleanSupplier> headingEnabled         = Optional.empty();
  /**
   * Locked heading for {@link SwerveInputMode#TRANSLATION_ONLY}
   */
  private       Optional<Rotation2d>      lockedHeading          = Optional.empty();
  /**
   * Output {@link ChassisSpeeds} based on aim while this is True.
   */
  private       Optional<BooleanSupplier> aimEnabled             = Optional.empty();
  /**
   * Maintain current heading and drive without rotating, ideally.
   */
  private       Optional<BooleanSupplier> translationOnlyEnabled = Optional.empty();
  /**
   * {@link SwerveController} for simple control over heading.
   */
  private       SwerveController          swerveController       = null;
  /**
   * Current {@link SwerveInputMode} to use.
   */
  private       SwerveInputMode           currentMode            = SwerveInputMode.ANGULAR_VELOCITY;


  /**
   * Drive modes to keep track of.
   */
  enum SwerveInputMode
  {
    /**
     * Translation only mode, does not allow for rotation and maintains current heading.
     */
    TRANSLATION_ONLY,
    /**
     * Output based off angular velocity
     */
    ANGULAR_VELOCITY,
    /**
     * Output based off of heading.
     */
    HEADING,
    /**
     * Output based off of targeting.
     */
    AIM
  }

  /**
   * Copy the {@link SwerveInputStream} object.
   *
   * @return Clone of current {@link SwerveInputStream}
   */
  public SwerveInputStream copy()
  {
    SwerveInputStream newStream = new SwerveInputStream(swerveDrive, controllerTranslationX, controllerTranslationY);
    newStream.controllerTheta = controllerTheta;
    newStream.controllerHeadingX = controllerHeadingX;
    newStream.controllerHeadingY = controllerHeadingY;
    newStream.axisDeadband = axisDeadband;
    newStream.translationScale = translationScale;
    newStream.angularVelocityScale = angularVelocityScale;
    newStream.aimTarget = aimTarget;
    newStream.headingEnabled = headingEnabled;
    newStream.aimEnabled = aimEnabled;
    newStream.currentMode = currentMode;
    newStream.translationOnlyEnabled = translationOnlyEnabled;
    newStream.lockedHeading = lockedHeading;
    newStream.swerveController = swerveController;
    return newStream;
  }

  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisSpeeds} from a driver controller.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     Translation X input in range of [-1, 1]
   * @param y     Translation Y input in range of [-1, 1]
   */
  private SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y)
  {
    controllerTranslationX = x;
    controllerTranslationY = y;
    swerveDrive = drive;
  }

  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisSpeeds} from a driver controller.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     Translation X input in range of [-1, 1]
   * @param y     Translation Y input in range of [-1, 1]
   * @param rot   Rotation input in range of [-1, 1]
   */
  public SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot)
  {
    this(drive, x, y);
    controllerTheta = Optional.of(rot);
  }

  /**
   * Create a {@link SwerveInputStream} for an easy way to generate {@link ChassisSpeeds} from a driver controller.
   *
   * @param drive    {@link SwerveDrive} object for transformation.
   * @param x        Translation X input in range of [-1, 1]
   * @param y        Translation Y input in range of [-1, 1]
   * @param headingX Heading X input in range of [-1, 1]
   * @param headingY Heading Y input in range of [-1, 1]
   */
  public SwerveInputStream(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y, DoubleSupplier headingX,
                           DoubleSupplier headingY)
  {
    this(drive, x, y);
    controllerHeadingX = Optional.of(headingX);
    controllerHeadingY = Optional.of(headingY);
  }

  /**
   * Create basic {@link SwerveInputStream} without any rotation components.
   *
   * @param drive {@link SwerveDrive} object for transformation.
   * @param x     {@link DoubleSupplier} of the translation X axis of the controller joystick to use.
   * @param y     {@link DoubleSupplier} of the translation X axis of the controller joystick to use.
   * @return {@link SwerveInputStream} to use as you see fit.
   */
  public static SwerveInputStream of(SwerveDrive drive, DoubleSupplier x, DoubleSupplier y)
  {
    return new SwerveInputStream(drive, x, y);
  }

  /**
   * Add a rotation axis for Angular Velocity control
   *
   * @param rot Rotation axis with values from [-1, 1]
   * @return self
   */
  public SwerveInputStream withRotation(DoubleSupplier rot)
  {
    controllerTheta = Optional.of(rot);
    return this;
  }

  /**
   * Add heading axis for Heading based control.
   *
   * @param headingX Heading X axis with values from [-1, 1]
   * @param headingY Heading Y axis with values from [-1, 1]
   * @return self
   */
  public SwerveInputStream withHeading(DoubleSupplier headingX, DoubleSupplier headingY)
  {
    controllerHeadingX = Optional.of(headingX);
    controllerHeadingY = Optional.of(headingY);
    return this;
  }

  /**
   * Set a deadband for all controller axis.
   *
   * @param deadband Deadband to set, should be between [0, 1)
   * @return self
   */
  public SwerveInputStream deadband(double deadband)
  {
    axisDeadband = deadband == 0 ? Optional.empty() : Optional.of(deadband);
    return this;
  }


  /**
   * Scale the axis inputs for {@link SwerveInputStream} to reduce the range in which they operate.
   *
   * @param scaleTranslation Translation axis scalar value. (0, 1]
   * @param scaleRotation    Angular velocity axis scalar value. (0, 1]
   * @return this.
   */
  public SwerveInputStream scale(double scaleTranslation, double scaleRotation)
  {
    translationScale = scaleTranslation == 0 ? Optional.empty() : Optional.of(scaleTranslation);
    angularVelocityScale = scaleRotation == 0 ? Optional.empty() : Optional.of(scaleRotation);
    return this;
  }


  /**
   * Output {@link ChassisSpeeds} based on heading while the supplier is True.
   *
   * @param trigger Supplier to use.
   * @return this.
   */
  public SwerveInputStream headingWhile(BooleanSupplier trigger)
  {
    headingEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Set the heading enable state.
   *
   * @param headingState Heading enabled state.
   * @return this
   */
  public SwerveInputStream headingWhile(boolean headingState)
  {
    if (headingState)
    {
      headingEnabled = Optional.of(() -> true);
    } else
    {
      headingEnabled = Optional.empty();
    }
    return this;
  }

  /**
   * Aim the {@link SwerveDrive} at this pose while driving.
   *
   * @param aimTarget {@link Pose2d} to point at.
   * @return this
   */
  public SwerveInputStream aim(Pose2d aimTarget)
  {
    this.aimTarget = aimTarget.equals(Pose2d.kZero) ? Optional.empty() : Optional.of(aimTarget);
    return this;
  }


  /**
   * Aim at an AprilTagID given by the current years layout, blue-origin only.
   *
   * @param aprilTagId AprilTag to point to.
   * @return this.
   */
  public SwerveInputStream aim(int aprilTagId)
  {
    Optional<Pose3d> aprilTagPose = field.getTagPose(aprilTagId);
    if (aprilTagPose.isPresent())
    {
      this.aimTarget = Optional.of(aprilTagPose.get().toPose2d());
    } else
    {
      this.aimTarget = Optional.empty();
      DriverStation.reportError("AprilTagID " + aprilTagId + " is not found!", true);
    }
    return this;
  }

  /**
   * Enable aiming while the trigger is true.
   *
   * @param trigger When True will enable aiming at the current target.
   * @return this.
   */
  public SwerveInputStream aimWhile(BooleanSupplier trigger)
  {
    aimEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Enable aiming while the trigger is true.
   *
   * @param trigger When True will enable aiming at the current target.
   * @return this.
   */
  public SwerveInputStream aimWhile(boolean trigger)
  {
    if (trigger)
    {
      aimEnabled = Optional.of(() -> true);
    } else
    {
      aimEnabled = Optional.empty();
    }
    return this;
  }

  /**
   * Enable locking of rotation and only translating, overrides everything.
   *
   * @param trigger Translation only while returns true.
   * @return this
   */
  public SwerveInputStream translationOnlyWhile(BooleanSupplier trigger)
  {
    translationOnlyEnabled = Optional.of(trigger);
    return this;
  }

  /**
   * Enable locking of rotation and only translating, overrides everything.
   *
   * @param translationState Translation only if true.
   * @return this
   */
  public SwerveInputStream translationOnlyWhile(boolean translationState)
  {
    if (translationState)
    {
      translationOnlyEnabled = Optional.of(() -> true);
    } else
    {
      translationOnlyEnabled = Optional.empty();
    }
    return this;
  }

  /**
   * Find {@link SwerveInputMode} based off existing parameters of the {@link SwerveInputStream}
   *
   * @return The calculated {@link SwerveInputMode}, defaults to {@link SwerveInputMode#ANGULAR_VELOCITY}.
   */
  private SwerveInputMode findMode()
  {
    if (translationOnlyEnabled.isPresent() && translationOnlyEnabled.get().getAsBoolean())
    {
      return SwerveInputMode.TRANSLATION_ONLY;
    } else if (aimEnabled.isPresent() && aimEnabled.get().getAsBoolean())
    {
      if (aimTarget.isPresent())
      {
        return SwerveInputMode.AIM;
      } else
      {
        DriverStation.reportError(
            "Attempting to enter AIM mode without target, please use SwerveInputStream.aim() to select a target first!",
            false);
      }
    } else if (headingEnabled.isPresent() && headingEnabled.get().getAsBoolean())
    {
      if (controllerHeadingX.isPresent() && controllerHeadingY.isPresent())
      {
        return SwerveInputMode.HEADING;
      } else
      {
        DriverStation.reportError(
            "Attempting to enter HEADING mode without heading axis, please use SwerveInputStream.withHeading to add heading axis!",
            false);
      }
    } else if (controllerTheta.isEmpty())
    {
      DriverStation.reportError(
          "Attempting to enter ANGULAR_VELOCITY mode without a rotation axis, please use SwerveInputStream.withRotation to add angular velocity axis!",
          false);
      return SwerveInputMode.TRANSLATION_ONLY;
    }
    return SwerveInputMode.ANGULAR_VELOCITY;
  }

  /**
   * Transition smoothly from one mode to another.
   *
   * @param newMode New mode to transition too.
   */
  private void transitionMode(SwerveInputMode newMode)
  {
    // Handle removing of current mode.
    switch (currentMode)
    {
      case TRANSLATION_ONLY ->
      {
        lockedHeading = Optional.empty();
        break;
      }
      case ANGULAR_VELOCITY ->
      {
        // Do nothing
        break;
      }
      case HEADING ->
      {
        // Do nothing
        break;
      }
      case AIM ->
      {
        // Do nothing
        break;
      }
    }

    // Transitioning to new mode
    switch (newMode)
    {
      case TRANSLATION_ONLY ->
      {
        lockedHeading = Optional.of(swerveDrive.getOdometryHeading());
        break;
      }
      case ANGULAR_VELOCITY ->
      {
        if (swerveDrive.headingCorrection)
        {
          swerveDrive.setHeadingCorrection(false);
        }
        break;
      }
      case HEADING ->
      {
        // Do nothing
        break;
      }
      case AIM ->
      {
        // Do nothing
        break;
      }
    }
  }

  /**
   * Apply the deadband if it exists.
   *
   * @param axisValue Axis value to apply the deadband too.
   * @return axis value with deadband, else axis value straight.
   */
  private double applyDeadband(double axisValue)
  {
    if (axisDeadband.isPresent())
    {
      return MathUtil.applyDeadband(axisValue, axisDeadband.get());
    }
    return axisValue;
  }

  /**
   * Apply the scalar value if it exists.
   *
   * @param axisValue Axis value to apply teh scalar too.
   * @param scalar    Scalar option to use.
   * @return Axis value scaled by scalar value.
   */
  private double applyScalar(double axisValue, Optional<Double> scalar)
  {
    if (scalar.isPresent())
    {
      return axisValue * scalar.get();
    }
    return axisValue;
  }

  /**
   * Gets a field-oriented {@link ChassisSpeeds}
   *
   * @return field-oriented {@link ChassisSpeeds}
   */
  @Override
  public ChassisSpeeds get()
  {
    double maximumChassisVelocity = swerveDrive.getMaximumChassisVelocity();
    double vxMetersPerSecond = applyScalar(applyDeadband(controllerTranslationX.getAsDouble()),
                                           translationScale) * maximumChassisVelocity;
    double vyMetersPerSecond = applyScalar(applyDeadband(controllerTranslationY.getAsDouble()),
                                           translationScale) * maximumChassisVelocity;
    double omegaRadiansPerSecond = 0;

    SwerveInputMode newMode = findMode();
    // Handle transitions here.
    if (currentMode != newMode)
    {
      transitionMode(newMode);
    }
    if (swerveController == null)
    {
      swerveController = swerveDrive.getSwerveController();
    }

    switch (newMode)
    {
      case TRANSLATION_ONLY ->
      {
        omegaRadiansPerSecond = swerveController.headingCalculate(swerveDrive.getOdometryHeading().getRadians(),
                                                                  lockedHeading.get().getRadians());
        break;
      }
      case ANGULAR_VELOCITY ->
      {
        omegaRadiansPerSecond = applyScalar(applyDeadband(controllerTheta.get().getAsDouble()), angularVelocityScale) *
                                swerveDrive.getMaximumChassisAngularVelocity();
        break;
      }
      case HEADING ->
      {
        omegaRadiansPerSecond = swerveController.headingCalculate(swerveDrive.getOdometryHeading().getRadians(),
                                                                  swerveController.getJoystickAngle(controllerHeadingX.get()
                                                                                                                      .getAsDouble(),
                                                                                                    controllerHeadingY.get()
                                                                                                                      .getAsDouble()));
        break;
      }
      case AIM ->
      {
        Rotation2d    currentHeading = swerveDrive.getOdometryHeading();
        Translation2d relativeTrl    = aimTarget.get().relativeTo(swerveDrive.getPose()).getTranslation();
        Rotation2d    target         = new Rotation2d(relativeTrl.getX(), relativeTrl.getY()).plus(currentHeading);
        omegaRadiansPerSecond = swerveController.headingCalculate(currentHeading.getRadians(), target.getRadians());
        break;
      }
    }

    currentMode = newMode;

    return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
  }
}
