package swervelib.simulation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import static swervelib.simulation.SwerveSimulationObjectsContainer.*;

/**
 * Class to hold simulation data for {@link swervelib.SwerveModule}
 */
public class SwerveModuleSimulation
{
  private final PIDController steerCloseLoop = new PIDController(3, 0, 0);

  private final org.ironmaple.simulation.drivesims.SwerveModuleSimulation simulation;
  public SwerveModuleSimulation(int number)
  {
    this.simulation = SwerveSimulationObjectsContainer.getInstance().getModules()[number];
    steerCloseLoop.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Update the position and state of the module. Called from {@link swervelib.SwerveModule#setDesiredState} function
   * when simulated.
   *
   * @param desiredState State the swerve module is set to.
   */
  public void updateStateAndPosition(SwerveModuleState desiredState)
  {
    final SwerveModuleState desiredStateOptimized = SwerveModuleState.optimize(
            desiredState,
            simulation.getSteerAbsoluteFacing()
    );
    final double velocityMPSProjected = desiredState.speedMetersPerSecond * desiredState.angle.minus(getState().angle).getCos(),
            driveMotorDesiredSpeedRadPerSec = velocityMPSProjected / WHEEL_RADIUS_METERS * DRIVE_GEAR_RATIO,
            driveVoltage = DRIVE_MOTOR.getVoltage(
                    0,
                    driveMotorDesiredSpeedRadPerSec
            );

    simulation.requestDriveVoltageOut(driveVoltage);
    simulation.requestSteerVoltageOut(steerCloseLoop.calculate(
            simulation.getSteerAbsoluteFacing().getRadians(),
            desiredStateOptimized.angle.getRadians()
    ));
  }

  /**
   * Get the simulated swerve module position.
   *
   * @return {@link SwerveModulePosition} of the simulated module.
   */
  public SwerveModulePosition getPosition()
  {

    return new SwerveModulePosition(
            simulation.getDriveWheelFinalPositionRad() * WHEEL_RADIUS_METERS,
            simulation.getSteerAbsoluteFacing()
    );
  }

  /**
   * Get the {@link SwerveModuleState} of the simulated module.
   *
   * @return {@link SwerveModuleState} of the simulated module.
   */
  public SwerveModuleState getState()
  {
    return new SwerveModuleState(
            simulation.getDriveWheelFinalSpeedRadPerSec() * WHEEL_RADIUS_METERS,
            simulation.getSteerAbsoluteFacing()
    );
  }
}
