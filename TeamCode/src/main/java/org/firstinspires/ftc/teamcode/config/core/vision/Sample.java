package org.firstinspires.ftc.teamcode.config.core.vision;

import static org.firstinspires.ftc.teamcode.config.core.RobotConstants.visionInch;

import org.firstinspires.ftc.teamcode.config.core.util.Color;

public class Sample {
    private Color color;
    private double x, y;
    private double angle;

    public Sample(Color color, double x, double y, double angle) {
        this.color = color;
        this.x = (x-640/2)/visionInch-3;
        this.y = ((y-360/2)/visionInch-5-7)*-1+0.5;
        this.angle = angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return angle;
    }

    public Color getColor() {
        return color;
    }
}