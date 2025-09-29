package org.firstinspires.ftc.robotcontroller.internal;

import android.app.Application;

import com.acmerobotics.dashboard.FtcDashboard;

public class FtcApplication extends Application {
    @Override
    public void onCreate() {
        super.onCreate();

        // Initialize FTC Dashboard
        FtcDashboard.getInstance();
    }
}