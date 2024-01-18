package frc.robot.subsystems

import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.io.File
import swervelib.SwerveController
import swervelib.SwerveDrive
import swervelib.parser.SwerveParser
import swervelib.telemetry.SwerveDriveTelemetry
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity

class Swerve : SubsystemBase() {

    var maximumSpeed = Units.feetToMeters(14.5)
    var swerveJsonDirectory = File(Filesystem.getDeployDirectory(), "swerve")
    var swerveDrive: SwerveDrive

    var frontleftCanCoder = CANcoder(33)
    var backleftCanCoder = CANcoder(35)
    var frontrightCanCoder = CANcoder(32)
    var backrightCanCoder = CANcoder(34)

    init {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH
        swerveDrive = SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed)
        swerveDrive.setHeadingCorrection(false)
    }

    /** This method will be called once per scheduler run */
    override fun periodic() {
        SmartDashboard.putNumber("front left", frontleftCanCoder.getAbsolutePosition().value)
        SmartDashboard.putNumber("front right", frontrightCanCoder.getAbsolutePosition().value)
        SmartDashboard.putNumber("back left", backleftCanCoder.getAbsolutePosition().value)
        SmartDashboard.putNumber("back right", backrightCanCoder.getAbsolutePosition().value)
        SmartDashboard.putNumber("pigeon", swerveDrive.yaw.degrees)
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
