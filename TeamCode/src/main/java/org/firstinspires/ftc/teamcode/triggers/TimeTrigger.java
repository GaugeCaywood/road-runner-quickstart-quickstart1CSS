package org.firstinspires.ftc.teamcode.triggers;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TimeTrigger extends Trigger {
    private final ElapsedTime runtime;
    private final double startTime;
    private final double endTime;

    public TimeTrigger(ElapsedTime runtime, double startTime, double endTime) {
        this.runtime = runtime;
        this.startTime = startTime;
        this.endTime = endTime;
    }

    @Override
    public boolean get() {
        double currentTime = runtime.seconds();
        return currentTime > startTime && currentTime < endTime;
    }
}
