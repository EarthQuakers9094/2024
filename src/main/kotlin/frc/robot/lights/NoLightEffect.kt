package frc.robot.lights

import edu.wpi.first.wpilibj.AddressableLEDBuffer

class NoLightEffect : LightEffect {
    override fun nextBuffer(tick: Int, addressableLEDBuffer: AddressableLEDBuffer) {
        for (i in 0 until addressableLEDBuffer.length) {
            addressableLEDBuffer.setRGB(i, 0, 0, 0)
        }
    }
}
