package frc.robot.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.DriverStation
import java.util.*
import java.util.function.Supplier
import com.revrobotics.REVLibError

fun Pose2d.toPose3d(): Pose3d {
    return Pose3d(this.x, this.y, 0.0, this.rotation.toRotation3d())
}

fun Rotation2d.toRotation3d(): Rotation3d {
    return Rotation3d(0.0, 0.0, this.radians)
}

fun <T> Optional<T>.toNullable(): T? {
    return if (this.isPresent) {
        this.get()
    } else {
        null
    }
}

fun configureSparkMax(config: Supplier<REVLibError> ) {
    for (i in 0..5) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
    }
    DriverStation.reportWarning("Failure configuring motor :3", true);
}