package frc.robot.subsystems

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.util.HolonomicPathFollowerConfig
import com.pathplanner.lib.util.ReplanningConfig
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import java.io.File
import java.util.function.Consumer
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonUtils
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.math.SwerveMath
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity
import frc.robot.utils.Config

class Swerve(private val camera: PhotonCamera) : SubsystemBase() {

    var maximumSpeed = Units.feetToMeters(14.5)
    var swerveJsonDirectory = File(Filesystem.getDeployDirectory(), Config("testswerve","swerve").config)
    var swerveDrive: SwerveDrive

    val robotToCam = Transform3d(Translation3d(0.5, 0.0, 0.5), Rotation3d(0.0, 0.0, 0.0))

    val aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField()
    //    val poseEstimator =
    //            PhotonPoseEstimator(
    //                    aprilTagFieldLayout,
    //                    PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
    //                    camera,
    //                    robotToCam
    //            )
    val lastEstimate: EstimatedRobotPose? = null


    var pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)

    init {
        // poseEstimator.referencePose = Pose3d()
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH
        swerveDrive = SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed)
        swerveDrive.setHeadingCorrection(false)

        var getPose = {
            var pose = this.getPos()
            SmartDashboard.putNumber("get pose pathplanner x moewjrek", pose.getX())
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
                        Constants.Drivebase.TRANSLATION_PID,
                        Constants.Drivebase.ROTATION_PID,
                        Constants.Drivebase.MAX_AUTO_SPEEDS,
                        Constants.Drivebase.RADIUS,
                        replanning
                )

        var pathConstraints =
                PathConstraints(
                        Constants.Drivebase.MAX_AUTO_SPEEDS,
                        Constants.Drivebase.MAX_ACCEL,
                        Constants.Drivebase.MAX_TURNING_SPEEDS,
                        Constants.Drivebase.MAX_ANGULAR_ACCELERATION,
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
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        // DriverStation.reportWarning("connected: " + camera.latestResult.targets.size, false)
        if (lastEstimate != null) {
            // poseEstimator.referencePose = lastEstimate!!.estimatedPose
        }
        val newEstimate =
        //                poseEstimator.update(camera.latestResult).let {
        //                    if (it.isPresent) it.get() else null
        //                }
        // DriverStation.reportWarning("estimate  " + newEstimate.toString(), false)
        //         val res = camera.latestResult
        //         if (res.hasTargets()) {
        //             PhotonUtils.estimateFieldToRobotAprilTag(
        //                     res.bestTarget.bestCameraToTarget,
        //                     aprilTagFieldLayout.getTagPose(res.bestTarget.fiducialId).get(),
        //                     robotToCam
        //             )
        //             val pose = res.bestTarget.bestCameraToTarget.plus(robotToCam.inverse())
        //             DriverStation.reportWarning("pose: " +
        // res.bestTarget.alternateCameraToTarget, false)
        //         }

        //        if (newEstimate != null) {
        //            DriverStation.reportError("new thing ", arrayOf())
        //            swerveDrive.addVisionMeasurement(
        //                    newEstimate.estimatedPose.toPose2d(),
        //                    Timer.getFPGATimestamp(),
        //                    Constants.Camera.camSTDEV
        //            )
        //            swerveDrive.resetOdometry(newEstimate.estimatedPose.toPose2d())
        //        }

        SmartDashboard.putNumber("pigeon", swerveDrive.yaw.degrees)

        var angleMotorConv = SwerveMath.calculateDegreesPerSteeringRotation(150.0 / 7.0, 1.0)
        // var angleMotorConv = SwerveMath.calculateDegreesPerSteeringRotation(150.0 / 7.0, 1.0)

        var driveConversionFactor =
                SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4.0), 6.12, 1.0)

        SmartDashboard.putNumber("angle conversion MEOW", angleMotorConv)
        SmartDashboard.putNumber("drive conversion MEOW", driveConversionFactor)
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        SmartDashboard.putNumber("pigeon", swerveDrive.yaw.degrees)



        //        SmartDashboard.putNumber("current: frontRight", pdh.getCurrent(8))
        //        SmartDashboard.putNumber("current: frontleft", pdh.getCurrent(10))
        //
        //        SmartDashboard.putNumber("current: backleft", pdh.getCurrent(17))
        //        SmartDashboard.putNumber("current: backright", pdh.getCurrent(1))
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
        val result = camera.latestResult
        if (result.hasTargets()) {
            val target = result.bestTarget

            val pos =
                    PhotonUtils.estimateFieldToRobotAprilTag(
                            target.bestCameraToTarget,
                            fieldRelativeTagPose,
                            Constants.Camera.cameraTransform
                    )
        }
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
