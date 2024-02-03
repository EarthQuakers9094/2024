package frc.robot

import com.pathplanner.lib.util.PIDConstants

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

        val TRANSLATION_PID = PIDConstants(6.0, 0.0, 0.0)
        val ROTATION_PID = PIDConstants(3.2, 0.0, 0.0)
    }
    object OperatorConstants {
        const val kDriverControllerPort = 0
        const val LEFT_X_DEADBAND = 0.055
        const val LEFT_Y_DEADBAND = 0.05
    }
    object Intake {
        const val speed = 0.6
        const val motorid = 24
        const val followMotorId = 25
    }
    object Camera {
        const val arducamOne = "Arducam1"
    }
    object Shooter {
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
        const val topCanid = 16
        const val bottomCanID = 17
        const val shooterJointCanID = 18
        const val shooterJoint2CanID = 19
    }

    object Elevator {
        val sim_pid = PIDConstants(5.0, 0.0, 0.0)
        val sim_feedforward = 0.125641
        val pid = PIDConstants(0.0, 0.0, 0.0)
        const val positionConversionFactor = 1.0
        const val gearing = 30.0
        const val motorID = 15
    }
}
