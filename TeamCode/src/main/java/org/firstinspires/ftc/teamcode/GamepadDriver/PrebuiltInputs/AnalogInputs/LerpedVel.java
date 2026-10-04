package org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.AnalogInputs;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadInput;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

import java.util.function.Function;

public class LerpedVel implements GamepadInput<Velocity>{
    GamepadInput<Velocity> rawVel;
    Velocity lastVel;
    Velocity lerpedVel;
    Scalar maxAccel;
    Scalar maxAngAccel;

    ElapsedTime tickTimer;

    public LerpedVel(GamepadInput<Velocity>rawVel, Scalar maxAccel, Scalar maxAngAccel){
        this.rawVel=rawVel;
        this.maxAccel=maxAccel;
        this.maxAngAccel=maxAngAccel;

        tickTimer=new ElapsedTime();
        lastVel=Velocity.zero;
    }

    public LerpedVel(Gamepad gamepad1, Gamepad gamepad2, Function<Gamepad,Double> drive, Function<Gamepad,Double> strafe, Function<Gamepad,Double> rot, Scalar velFactor, Scalar angFactor, Scalar maxAccel, Scalar maxAngAccel){
        this(new DualDirectVel(gamepad1,gamepad2,drive,strafe,rot,velFactor,angFactor),maxAccel,maxAngAccel);
    }

    @Override
    public void update() {
        Velocity targetVel=rawVel.get();
        Scalar maxDelta=maxAccel.multiply(new Scalar(tickTimer.seconds(),s));
        Scalar maxAngDelta=maxAngAccel.multiply(new Scalar(tickTimer.seconds(),s));
        Velocity delta=targetVel.sub(lastVel);

        lerpedVel=new Velocity(delta.mag().lessThanOrEqual(maxDelta)?targetVel.getLinVel():lastVel.getLinVel().add(delta.unitVector().multi(maxDelta)),
                delta.getAngVel().abs().lessThanOrEqual(maxAngDelta)?targetVel.getAngVel():lastVel.getAngVel().div(lastVel.getAngVel().abs()).multiply(maxAngDelta));
    }

    @Override
    public Velocity get() {
        return lerpedVel;
    }
}
