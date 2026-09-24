package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

// Watches a ColorSensing sensor and drives the indexer servo based on what it sees:
// yellow (or nothing reliably detected) -> starting position, red/blue -> wider position.
public class PieceDetector {
    private static final double STARTING_POSITION = 0.0;
    private static final double WIDER_POSITION = 1.0;

    private final ColorSensing colorSensing;
    private final Servo servo;

    public PieceDetector(ColorSensing colorSensing, Servo servo) {
        this.colorSensing = colorSensing;
        this.servo = servo;
    }

    // Call this every loop. Reads the sensor and moves the servo to match.
    public void update() {
        ColorSensing.GamePiece piece = colorSensing.classify();
        switch (piece) {
            case NECTAR_RED:
            case NECTAR_BLUE:
                servo.setPosition(WIDER_POSITION);
                break;
            case POLLEN_YELLOW:
            case NONE:
            default:
                servo.setPosition(STARTING_POSITION);
                break;
        }
    }
}
