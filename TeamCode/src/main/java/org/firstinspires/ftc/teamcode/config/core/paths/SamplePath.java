package org.firstinspires.ftc.teamcode.config.core.vision;

import com.pedropathing.pathgen.PathChain;

public class SamplePath {

    public final PathChain path;
    public final Sample sample;
    public final double deltaX;
    public final double lateralMove;
    public SamplePath(PathChain path, Sample sample, double deltaX, double lateralMove) {
        this.path = path;
        this.sample = sample;
        this.deltaX = deltaX;
        this.lateralMove = lateralMove;
    }
}