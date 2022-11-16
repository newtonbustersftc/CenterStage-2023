package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class TestEncoder {
    int count;
    private Encoder.Direction direction;
    public enum Direction {
        FORWARD(1),
        REVERSE(-1);

        private int multiplier;

        Direction(int multiplier) {
            this.multiplier = multiplier;
        }

        public int getMultiplier() {
            return multiplier;
        }
    }

    public TestEncoder(int count){
        this.count = count;
    }

    public int getCurrentPosition(){
        return count;
    }

    public void setDirection(Encoder.Direction direction) {
        this.direction = direction;
    }

    public double getRawVelocity() {
        return 0;
    }
}
