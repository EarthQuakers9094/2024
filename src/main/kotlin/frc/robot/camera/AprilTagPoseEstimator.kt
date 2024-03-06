package frc.robot.camera

import edu.wpi.first.hal.HALUtil
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.apriltag.AprilTag
import frc.robot.Constants
import frc.robot.subsystems.Swerve
import frc.robot.utils.toNullable
import frc.robot.utils.toPose3d
import frc.robot.camera.AprilTagResult
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.EstimatedRobotPose
import swervelib.SwerveDrive

class AprilTagPoseEstimator(val swerve: SwerveDrive, val camera: PhotonCamera, private val offset: Transform3d) {

    private val poseEstimator = PhotonPoseEstimator(
        Constants.Camera.aprilTagFieldLayout,
        PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        offset
    )



    init {
        poseEstimator.referencePose = swerve.pose.toPose3d()
    }

    fun update(): AprilTagResult {
        if (!camera.isConnected) {
            DriverStation.reportWarning("Camera with name: ${camera.name} is not connected", false)
            return AprilTagResult(null, Timer.getFPGATimestamp(), 0)
        }
        val timestamp = Timer.getFPGATimestamp()
        val targets = camera.latestResult.targets
        

        if(targets.all { it.poseAmbiguity <= 0.5 }) {
            return AprilTagResult(poseEstimator.update().toNullable(), timestamp, targets.size)
        }
        
        return AprilTagResult(null, Timer.getFPGATimestamp(), 0)


    }
        
        // AprilTagResult(, camera.latestResult.targets.size)/*?.let {   
                
    
    
}