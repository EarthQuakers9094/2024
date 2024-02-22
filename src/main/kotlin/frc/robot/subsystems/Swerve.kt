package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.camera.AprilTagPoseEstimator
import frc.robot.utils.Config
import frc.robot.utils.toNullable
import frc.robot.utils.toPose3d
import java.io.File
import java.util.function.Consumer
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.math.SwerveMath
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity

class Swerve() : SubsystemBase() {

    var maximumSpeed = Units.feetToMeters(14.5)
    var swerveJsonDirectory =
            File(Filesystem.getDeployDirectory(), Config("testswerve", "swerve").config)
    var swerveDrive: SwerveDrive = SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed)

    var frontleftCanCoder = CANcoder(11)
    var backleftCanCoder = CANcoder(12)
    var frontrightCanCoder = CANcoder(13)
    var backrightCanCoder = CANcoder(14)

    var pdh = PowerDistribution(9, PowerDistribution.ModuleType.kRev)

    private val poseEstimators = arrayOf(AprilTagPoseEstimator(swerveDrive, PhotonCamera("BW"), Constants.Camera.cameraTransform))

    init {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH
        swerveDrive.setHeadingCorrection(false)


        var getPose = {
            var pose = this.getPos()
            SmartDashboard.putNumber("get pose pathplanner x", pose.getX())
            SmartDashboard.putNumber("get pose pathplanner y", pose.getY())
            SmartDashboard.putNumber("get pose pathplanner rotation", pose.rotation.degrees)

            pose
        }

        var setPose: Consumer<Pose2d> = Consumer { pose -> this.resetOdomentry(pose) }

        var getSpeeds = { this.getSpeeds() }
        var drive: Consumer<ChassisSpeeds> = Consumer { speeds ->
            SmartDashboard.putNumber("path planner direction x", speeds.vxMetersPerSecond)
            SmartDashboard.putNumber("path planner direction y", speeds.vyMetersPerSecond)
            SmartDashboard.putNumber(
                    "path planner direction rotationSpeed",
                    speeds.omegaRadiansPerSecond
            )

            this.driveRobotRel(speeds)
        }

        var replanning = ReplanningConfig()

        var config =
                HolonomicPathFollowerConfig(
                        Constants.Drivebase.TRANSLATION_PID.config,
                        Constants.Drivebase.ROTATION_PID.config,
                        Constants.Drivebase.MAX_AUTO_SPEEDS,
                        Constants.Drivebase.RADIUS,
                        replanning
                )

        var flip = {
            // Boolean supplier that controls when the path will be mirrored for the
            // red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance()
            var ret = false
            if (alliance.isPresent()) {
                ret = alliance.get() == DriverStation.Alliance.Red
            }
            ret
        }

        AutoBuilder.configureHolonomic(
                getPose, // Robot pose supplier
                setPose, // Method to reset odometry (will be called if your auto
                // has a starting pose)
                getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                drive, // Method that will drive the robot given ROBOT RELATIVE
                // ChassisSpeeds
                config,
                flip,
                this // Reference to this subsystem to set requirements
        )
        val driveConversionFactor =
                SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4.15686), 6.12, 1.0)

        SmartDashboard.putNumber("drive conversion factor", driveConversionFactor)
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {

        poseEstimators.forEach(AprilTagPoseEstimator::update)
        SmartDashboard.putNumber("pigeon", swerveDrive.yaw.degrees)

        // SmartDashboard.putNumber("current: frontRight", pdh.getCurrent(8))
        // SmartDashboard.putNumber("current: frontleft", pdh.getCurrent(10))

        // SmartDashboard.putNumber("current: backleft", pdh.getCurrent(17))
        // SmartDashboard.putNumber("current: backright", pdh.getCurrent(1))

    }

    fun drive(translation: Translation2d, rotation: Double, fieldRelative: Boolean) {
        swerveDrive.drive(
                translation,
                rotation,
                fieldRelative,
                false
        ) // Open loop is disabled since it shouldn't be used most of the time.
    }

    fun driveRobotRel(speeds: ChassisSpeeds) {
        drive(
                Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond,
                false
        )
    }

    fun getPos(): Pose2d {
        return swerveDrive.pose
    }

    fun zeroGyro() {
        swerveDrive.zeroGyro()
    }

    fun resetOdomentry(pose: Pose2d) {
        swerveDrive.resetOdometry(pose)
    }

    fun setMotorBrake(enabled: Boolean) {
        swerveDrive.setMotorIdleMode(enabled)
    }

    fun getSwerveController(): SwerveController {
        return swerveDrive.swerveController
    }

    fun getSpeeds(): ChassisSpeeds {
        return swerveDrive.fieldVelocity
    }

    /** This method will be called once per scheduler run during simulation */
    override fun simulationPeriodic() {}
}
