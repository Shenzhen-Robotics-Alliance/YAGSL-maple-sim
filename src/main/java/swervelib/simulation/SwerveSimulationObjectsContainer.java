package swervelib.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import java.util.Arrays;

public class SwerveSimulationObjectsContainer {
    protected static DCMotor
            DRIVE_MOTOR = DCMotor.getKrakenX60(1),
            STEER_MOTOR = DCMotor.getFalcon500(1);
    protected static double
            DRIVE_GEAR_RATIO = 6.75,
            WHEEL_RADIUS_METERS = Units.inchesToMeters(2),
            STEER_GEAR_RATIO = 21.4,
            MASS_KG = 45,
            DRIVE_CURRENT_LIMIT = 60,
            WHEEL_COF = 1.2,
            BUMPERS_WIDTH_METERS = Units.inchesToMeters(27),
            BUMPERS_LENGTH_METERS = Units.inchesToMeters(27);

    private static Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(12)),
            new Translation2d(Units.inchesToMeters(12), -Units.inchesToMeters(12)),
            new Translation2d(-Units.inchesToMeters(12), Units.inchesToMeters(12)),
            new Translation2d(-Units.inchesToMeters(12), -Units.inchesToMeters(12))
    };

    public static void configureDriveMotor(DCMotor driveMotorType) {
        DRIVE_MOTOR = driveMotorType;
    }

    public static void configureSteerMotor(DCMotor steerMotorType) {
        STEER_MOTOR = steerMotorType;
    }

    public static void configureRobotMass(double massWithBumpersKg) {
        MASS_KG = massWithBumpersKg;
    }

    public static void configureWheelRadius(double wheelRadiusMeters) {
        WHEEL_RADIUS_METERS = wheelRadiusMeters;
    }

    public static void configureDriveGearRatio(double driveGearRatio) {
        MASS_KG = driveGearRatio;
    }

    public static void configureBumperSize(double bumperWidthMeters, double bumperLengthMeters) {
        BUMPERS_WIDTH_METERS = bumperWidthMeters;
        BUMPERS_LENGTH_METERS = bumperLengthMeters;
    }

    public static void feedConfigs(double steerConversationRatio, double driveMotorCurrentLimit, double wheelGrippingCOF) {
        STEER_GEAR_RATIO = steerConversationRatio;
        DRIVE_CURRENT_LIMIT = driveMotorCurrentLimit;
        WHEEL_COF = wheelGrippingCOF;
    }
    public static SwerveModuleSimulation getModule(Translation2d moduleLocation) {
        System.out.println("drive gear ratio: " + DRIVE_GEAR_RATIO);
        return new SwerveModuleSimulation(
                DRIVE_MOTOR,
                STEER_MOTOR,
                DRIVE_CURRENT_LIMIT,
                DRIVE_GEAR_RATIO,
                STEER_GEAR_RATIO,
                0.15,
                0.15,
                WHEEL_COF,
                WHEEL_RADIUS_METERS,
                0.03
        );
    }

    private static SwerveDriveSimulation instance = null;
    public static SwerveDriveSimulation getInstance() {
        if (instance != null) return instance;

        instance = new SwerveDriveSimulation(
                MASS_KG,
                BUMPERS_WIDTH_METERS,
                BUMPERS_LENGTH_METERS,
                Arrays.stream(MODULE_TRANSLATIONS).map(
                        SwerveSimulationObjectsContainer::getModule
                ).toArray(SwerveModuleSimulation[]::new),
                MODULE_TRANSLATIONS,
                SwerveIMUSimulation.getInstance(),
                new Pose2d(3, 3, new Rotation2d())
        );
        SimulatedArena.getInstance().addDriveTrainSimulation(instance);

        return getInstance();
    }
}
