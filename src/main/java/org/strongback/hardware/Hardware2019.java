package org.strongback.hardware;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.strongback.components.Motor;
import org.strongback.function.DoubleToDoubleFunction;
import org.strongback.util.Values;

public class Hardware2019 {

    private static final DoubleToDoubleFunction SPEED_LIMITER = Values.limiter(-1.0, 1.0);

    public static Motor victorSPX(int channel) {
        return new HardwareMotor(new WPI_VictorSPX(channel), SPEED_LIMITER);
    }

}
