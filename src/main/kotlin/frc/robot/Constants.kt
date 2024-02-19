package frc.robot

import com.pathplanner.lib.util.PIDConstants
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import frc.robot.utils.Config

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. inside the companion object). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
object Constants {
    object Drivebase {
        const val WHEEL_LOCK_TIME = 10.0
        const val MAX_AUTO_SPEEDS = 3.0
        const val RADIUS = 0.6

        const val MAX_ACCEL = 1.0
        const val MAX_ANGULAR_ACCELERATION = 0.5
        const val MAX_TURNING_SPEEDS = 9.0

        const val compensation = 0.0;

        val TRANSLATION_PID = Config(PIDConstants(0.8, 0.0, 0.4), PIDConstants(4.0, 0.0, 0.4))
        // val ROTATION_PID = PIDConstants(3.2, 0.0, 0.0)
        val ROTATION_PID = Config(PIDConstants(3.2, 0.0, 0.0), PIDConstants(1.0, 0.0, 0.5))

        val ROTATION_PID_TELEOP = PIDConstants(0.0, 0.1, 0.0)
    }
    object OperatorConstants {
        const val kDriverControllerPort = 2
        const val driverLeftStickPort = 0
        const val driverRightStickPort = 1
        const val LEFT_X_DEADBAND = 0.1
        const val LEFT_Y_DEADBAND = 0.1
    }
    object Intake {
        const val speed = -0.60
        // const val speed = 0.0
        const val motorid = 24
        const val followMotorId = 25
    }
    object Camera {
        const val arducamOne = "Arducam1"
        val camSTDEV = run {
            val mat = Matrix(Nat.N3(), Nat.N1())
            mat.set(0,0, 1.0)
            mat.set(1,0, 1.0)
            mat.set(2,0, 1.0)
            mat
        }
        val cameraTransform = Transform3d(Translation3d(0.0, 0.0, 0.0), Rotation3d(0.0, 0.0, 0.0))
        val aprilTagFieldLayout = AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile)
    }
    object Auto {
        val TARGET_ROTATION = PIDConstants(0.03, 0.0, 0.0)
    }


    object Shooter {

        const val angleOffset = 0.0
        const val distanceOffset = 0.0
        const val heightOffset = 0.0
        const val mainShooterID = -1
        const val followerShooterID = -1
        const val p = 0.0
        const val i = 0.0
        const val d = 0.0
        val join_pid = PIDConstants(0.0, 0.0, 0.0)
        val sim_join_pid = PIDConstants(10.0, 0.0, 0.0)

        val sim_pid = PIDConstants(20.0, 0.0, 0.0)
        const val velocityConversionFactor = 1.0
        const val positionConversionFactor = 2.0 / 30.0 * Math.PI
        const val tolerance = 30.0
        const val topCanid = 26
        const val bottomCanID = 27
        const val shooterJointCanID = 28
        const val shooterJoint2CanID = 29
        const val intakeSpeed = 0.9
        const val speed = -0.75
        const val intakeMotorID = 30
        const val spinuptime = 5.0
        const val shootTime = 0.6
        const val closestDistance = 200
        val validTargets = arrayOf(4, 3);
        const val ampSpeed = 0.2;
        const val ampShootingRotationSpeed = 0.0;
    }

        val validTargets = arrayOf(4, 3)
    object Elevator {
        val sim_pid = PIDConstants(5.0, 0.0, 0.0)
        val sim_feedforward = 0.125641
        val pid = PIDConstants(0.0, 0.0, 0.0)
        const val positionConversionFactor = 1.0
        const val gearing = 30.0
        const val motorID = 15
        const val maxHeight = 1.0;
    }

    object Poses {
        object Amp {
            const val angle = -Math.PI/4.0;
            const val height = 0.2;
        }
    }
}
