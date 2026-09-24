package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensing {
    // Game pieces this sensor needs to tell apart
    public enum GamePiece {
        POLLEN_YELLOW,
        NECTAR_RED,
        NECTAR_BLUE,
        NONE // nothing close enough to read reliably
    }

    private NormalizedColorSensor sensor;
    private final float[] hsvValues = new float[3];

    // Hue ranges (degrees, 0-360), centered on values measured on-robot 2026-09-17:
    // yellow ball -> 60deg, red ball -> 0deg, blue ball -> 180deg (not the ~240 you'd expect
    // from a color wheel - re-verify with a higher gain before trusting this range long-term).
    private static final float YELLOW_HUE_MIN = 40, YELLOW_HUE_MAX = 75;
    private static final float BLUE_HUE_MIN = 150, BLUE_HUE_MAX = 210;
    // Red wraps around 0/360, so it's checked as hue < RED_HUE_MAX or hue > RED_HUE_MIN_HIGH
    private static final float RED_HUE_MAX = 25, RED_HUE_MIN_HIGH = 330;

    // How close (cm) a piece needs to be before we trust a color reading.
    // Measured on-robot 2026-09-17: sensor reports ~7cm at an actual 2-inch (5.08cm) gap -
    // that offset is normal for this sensor, so the cutoff is set from the *reported* value.
    private static final double MAX_READ_DISTANCE_CM = 8.0;

    // Below this, the ball is too close and the sensor's own LED oversaturates the
    // reading, skewing hue toward red (seen misreading yellow as red at ~0.5in and closer).
    // The accurate zone is roughly 0.5in-1in - TODO: watch the "Distance (cm)" telemetry
    // in PieceDetectorTest at ~0.5in on-robot and replace this with the *reported* value.
    private static final double MIN_READ_DISTANCE_CM = 0.0;

    // Below this, R/G/B readings are noise, not real color signal (measured on-robot)
    private static final float MIN_RELIABLE_COLOR = 0.1f;

    // Multiplies raw sensor values before normalizing. REV sensors read near-zero RGB
    // at gain 1 under normal room lighting. Gain 8 still only produced 0.02-0.07 on-robot
    // at 2in (2026-09-17) - start much higher and tune with ColorSensingTest.
    private static final float DEFAULT_GAIN = 20;

    public ColorSensing(NormalizedColorSensor sensor) {
        this.sensor = sensor;
        sensor.setGain(DEFAULT_GAIN);
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(true);
        }
    }

    public void setGain(float gain) {
        sensor.setGain(gain);
    }

    public float getGain() {
        return sensor.getGain();
    }

    public NormalizedRGBA getRawColors() {
        return sensor.getNormalizedColors();
    }

    public float getHue() {
        Color.colorToHSV(getRawColors().toColor(), hsvValues);
        return hsvValues[0];
    }

    public double getDistanceCM() {
        if (sensor instanceof DistanceSensor) {
            return ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);
        }
        return -1; // sensor doesn't support distance
    }

    // True if the current reading is strong enough to trust (not too far, not too dim)
    public boolean isReadingReliable() {
        double distanceCM = getDistanceCM();
        if (distanceCM > MAX_READ_DISTANCE_CM || distanceCM < MIN_READ_DISTANCE_CM) {
            return false;
        }
        NormalizedRGBA raw = getRawColors();
        float strongest = Math.max(raw.red, Math.max(raw.green, raw.blue));
        return strongest >= MIN_RELIABLE_COLOR;
    }

    public GamePiece classify() {
        if (!isReadingReliable()) {
            return GamePiece.NONE;
        }

        float hue = getHue();
        if (hue >= YELLOW_HUE_MIN && hue <= YELLOW_HUE_MAX) {
            return GamePiece.POLLEN_YELLOW;
        }
        if (hue >= BLUE_HUE_MIN && hue <= BLUE_HUE_MAX) {
            return GamePiece.NECTAR_BLUE;
        }
        if (hue <= RED_HUE_MAX || hue >= RED_HUE_MIN_HIGH) {
            return GamePiece.NECTAR_RED;
        }
        return GamePiece.NONE;
    }
}
