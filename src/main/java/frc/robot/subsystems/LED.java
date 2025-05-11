package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public final class LED extends SubsystemBase implements NiceSubsystem {
    private static LED instance;

    private final CANdle candle;

    private LED() {
        candle = new CANdle(Constants.LEDConstants.CANDLE_ID);

        CANdleConfiguration lightcon = new CANdleConfiguration();

        lightcon.statusLedOffWhenActive = false;
        lightcon.disableWhenLOS = false;
        lightcon.stripType = LEDStripType.RGB;
        lightcon.brightnessScalar = 1;
        lightcon.vBatOutputMode = VBatOutputMode.Modulated;
    }

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    public void setAllLEDToColor(int r, int g, int b) {
        candle.setLEDs(r, g, b);
        setModulatedOutput(0.9);
    }

    public void setOneLedToColor(int[] rgb, int ledNumber) {
        candle.setLEDs(rgb[0], rgb[1], rgb[2], 0, ledNumber, 1);
        setModulatedOutput(0.9);
    }

    public void doAnimate(Animation animation) {
        candle.animate(animation);
    }

    public void dontAnimate() {
        candle.clearAnimation(0);
    }

    public void setLEDOff() {
        candle.setLEDs(0, 0, 0);
        setModulatedOutput(0);
    }

    private void setModulatedOutput(double modulateVBatOutput) {
        candle.modulateVBatOutput(modulateVBatOutput);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {

    }

}
