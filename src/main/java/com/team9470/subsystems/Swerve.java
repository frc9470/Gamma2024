package com.team9470.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team9470.subsystems.vision.VisionDevice;
import com.team9470.subsystems.vision.VisionPoseAcceptor;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.photonvision.EstimatedRobotPose;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.function.DoubleSupplier;

import static com.team9470.Constants.*;

public class Swerve extends SubsystemBase {
    private static Swerve instance;
    private final SwerveDrive swerveDrive;

    private final VisionPoseAcceptor visionPoseAcceptor = new VisionPoseAcceptor();

    public Swerve() { // 80% of free speed // TODO: this doesn't really look right tbh
        File f = new File(Filesystem.getDeployDirectory(), "swerve");
        SwerveDriveTelemetry.verbosity = SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
        try {
            this.swerveDrive = new SwerveParser(f).createSwerveDrive(SwerveConstants.MAX_SPEED);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        configPathPlanner();
    }

    public static Swerve getInstance(){
        if(instance == null){
            instance = new Swerve();
        }
        return instance;
    }

    private void configPathPlanner() {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotVelocity,
                this::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        AutonConstants.TRANSLATION_PID,
                        AutonConstants.ROTATION_PID,
                        SwerveConstants.MAX_SPEED,
                        swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                        new ReplanningConfig()
                ),
                Swerve::isRedAlliance,
                this
        );
    }

//    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularSpeedX){
//        return this.run(() ->
//                swerveDrive.drive(
//                        new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumVelocity(), translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
//                        angularSpeedX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
//                        true,
//                        false
//                )
//        );
//    }

    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularSpeedX){
        return this.run(() ->
                swerveDrive.drive(
                        new Translation2d(translationX.getAsDouble() * 0.1, translationY.getAsDouble() * 0.1),
                        angularSpeedX.getAsDouble() * 0.1,
                        true,
                        false
                )
        );
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.

    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
    }

    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new SysIdRoutine.Config(),
                        this, swerveDrive, 12),
                3.0, 5.0, 3.0);
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand()
    {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new SysIdRoutine.Config(),
                        this, swerveDrive),
                3.0, 5.0, 3.0);
    }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }


    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }

    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }

    /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is available.
     */
    public static boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public void zeroGyroWithAlliance()
    {
        if (isRedAlliance())
        {
            zeroGyro();
            //Set the pose 180 degrees
            resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else
        {
            zeroGyro();
        }
    }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake)
    {
        swerveDrive.setMotorIdleMode(brake);
    }


    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity()
    {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Get Twist2d of robot velocity
     */
    public Twist2d getRobotTwist(){
        return new Twist2d(getRobotVelocity().vxMetersPerSecond, getRobotVelocity().vyMetersPerSecond, getRobotVelocity().omegaRadiansPerSecond);
    }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController()
    {
        return swerveDrive.swerveController;
    }


    public void addVisionMeasurement(Pose2d cameraPose, double timestamp, Matrix<N3, N1> stddevs) {
        if(visionPoseAcceptor.shouldAcceptVision(timestamp, cameraPose, getPose(), getRobotTwist(), DriverStation.isAutonomous()))
            swerveDrive.addVisionMeasurement(cameraPose, timestamp, stddevs);

    }
}
