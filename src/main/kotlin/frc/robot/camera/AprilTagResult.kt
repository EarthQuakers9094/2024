package frc.robot.camera

import org.photonvision.EstimatedRobotPose
data class AprilTagResult(val estimatedPose: EstimatedRobotPose?, val timestamp: Double, val targets: Int)