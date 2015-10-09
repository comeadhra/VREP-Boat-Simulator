package vrepSim;

import com.madara.KnowledgeBase;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

/**
 * @author jjb
 */
public class BoatMotionController implements VelocityProfileListener {

    int stateSize;
    RealMatrix x;
    RealMatrix xd;
    RealMatrix profile; // current velocity profile to follow
    RealMatrix xError;
    RealMatrix xErrorOld; // used for derivative control
    RealMatrix xErrorDiff; // used for derivative control
    KnowledgeBase knowledge;
    Long t;
    Long tOld;
    double dt;
    double t0; // time at the start of the velocity profile
    double tRampStart;
    double tRampCurrent;
    boolean t0set;
    boolean pointing;
    LutraMadaraContainers containers;
    final double headingErrorThreshold = 20.0*Math.PI/180.0; // +/- 20 deg
    final double returnToPointingThreshold = 45.0*Math.PI/180.0; // +/- 45 deg
    double simplePIDGains[][];
    double PPIGains[];
    double PPIErrorAccumulator; // [Pos-P*(pos error) + vel error] accumulation
    double[] simplePIDErrorAccumulator; // cols: x,y,th
    public static final double SAFE_DIFFERENTIAL_THRUST = 0.2;
    public static final double MIN_DIFFERENTIAL_BEARING = 0.0;
    public static final double MAX_DIFFERENTIAL_BEARING = 0.2;
    public static final double SAFE_VECTORED_THRUST = 0.6;
    double headingSignal = 0.0;
    double thrustSignal = 0.0;
    DatumListener datumListener;
    VelocityMotorMap velocityMotorMap;

    public BoatMotionController(KnowledgeBase knowledge, BoatEKF boatEKF, LutraMadaraContainers containers) {
        this.knowledge = knowledge;
        this.containers = containers;
        this.stateSize = boatEKF.stateSize;
        x = MatrixUtils.createRealMatrix(this.stateSize,1);
        xError = MatrixUtils.createRealMatrix(3,1); // [x y th w]
        xErrorOld = MatrixUtils.createRealMatrix(3,1); // [x y th w]
        xd = MatrixUtils.createRealMatrix(2,1); // [x y]
        PPIErrorAccumulator = 0;
        simplePIDErrorAccumulator = new double[]{0,0,0};
        t = System.currentTimeMillis();
        datumListener = boatEKF;
        velocityMotorMap = new VelocityMotorMap(containers);
        PPIGains = new double[3];
        simplePIDGains = new double[2][3];
        pointing = true;
    }

    public void zeroErrors() {
        simplePIDErrorAccumulator = new double[] {0.0,0.0,0.0};
        PPIErrorAccumulator = 0.0;
    }

    public void control() {
        updateFromKnowledgeBase();
        if (containers.teleopStatus.get() == TELEOPERATION_TYPES.NONE.getLongValue()) {
            xErrorOld = xError.copy();
            tOld = t;
            t = System.currentTimeMillis();
            dt = (t.doubleValue()-tOld.doubleValue())/1000.0;

            // current position error
            xError.setSubMatrix(xd.getSubMatrix(0, 1, 0, 0).subtract(x.getSubMatrix(0, 1, 0, 0)).getData(), 0, 0);

            // current heading error
            double angleToGoal = Math.atan2(xError.getEntry(1,0),xError.getEntry(0,0));
            double angleError = x.getEntry(2,0) - angleToGoal;

            // Error magnitude must be <= 180 degrees. Wrap the error into [-180,180]
            while (Math.abs(angleError) > Math.PI) {
                angleError = angleError - Math.signum(angleError)*2*Math.PI;
            }

            java.lang.String angleErrorString = java.lang.String.format("th = %f  thd = %f  ERROR = %f [deg]",
                    x.getEntry(2,0)*180.0/Math.PI,angleToGoal*180.0/Math.PI,angleError*180.0/Math.PI);
            //Log.i("jjb_ANGLEERROR",angleErrorString);

            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            double velToGoal = containers.velocityTowardGoal();
            double rotVel = x.getEntry(3, 0)*180.0/Math.PI;
            if (pointing) {
                //if (velToGoal > 0.5 && Math.abs(rotVel) < 5.0) {
                if (Math.abs(rotVel) < 4.0 && Math.abs(angleError) < headingErrorThreshold) {
                    pointing = false;
                    zeroErrors(); // start integration of error with shooting status
                }
            }
            //Log.i("jjb_POINTING", String.format("angleError = %.4f [deg]  vel toward goal = %.4f [m/s]   rotVel = %.4f [deg/s]   STATUS = %s", angleError * 180.0 / Math.PI, velToGoal, rotVel, (pointing ? "pointing" : "shooting")));
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if (Math.abs(angleError) > returnToPointingThreshold) { // the only two ways to point is to arrive at your goal after shooting or if your angle error is very large
                pointing = true;
            }

            xError.setEntry(2, 0, angleError);

            double distToDestCalc = RMO.distance(x.getSubMatrix(0, 1, 0, 0), xd.getSubMatrix(0, 1, 0, 0));

            containers.distToDest.set(distToDestCalc);

            //Log.i("jjb_XD","xd = " + RMO.realMatrixToString(xd));
            //Log.i("jjb_X","x = " + RMO.realMatrixToString(x));
            //Log.i("jjb_DISTTODEST",String.format("distToDest CALC = %.3f  distToDest CONTAINER = %.3f  sufficientProximity = %.3f",distToDestCalc,containers.distToDest.get(),containers.sufficientProximity.get()));

            if (containers.distToDest.get() < containers.sufficientProximity.get()) {
                containers.executingProfile.set(0);
                // reset PPICascade and simplePID accumulated error variables
                zeroErrors();
                thrustSignal = 0.0;
                headingSignal = 0.0;
                if (!pointing) {
                    //Log.w("jjb","*** ARRIVED AT GOAL ***");
                }
                pointing = true; // the only two ways to point is to arrive at your goal after shooting or if your angle error is very large
            }
            else {
                for (int i = 0; i < 3; i++) {
                    simplePIDErrorAccumulator[i] += xError.getEntry(i, 0) * dt;
                }
                xErrorDiff = (xError.subtract(xErrorOld)).scalarMultiply(1.0 / dt);

                String xErrorDiffString = String.format("xErrorDiff = %s", RMO.realMatrixToString(xErrorDiff));
                //Log.i("jjb_XERRORDIFF",xErrorDiffString);

                double Pterm = 0.0;
                double Iterm = 0.0;
                double Dterm = 0.0;
                if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
                    Pterm = simplePIDGains[1][0] * xError.getEntry(2, 0);
                    if (pointing) {
                        Dterm = simplePIDGains[1][2] * xErrorDiff.getEntry(2, 0);
                    } else {
                        Iterm = simplePIDGains[1][1] * simplePIDErrorAccumulator[2];
                    }
                }
                else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
                    if (pointing) {
                        Pterm = simplePIDGains[1][0] * xError.getEntry(2, 0);
                        Dterm = simplePIDGains[1][2] * xErrorDiff.getEntry(2, 0);
                    } else {
                        Pterm = 0.1*simplePIDGains[1][0] * xError.getEntry(2, 0);
                        Iterm = simplePIDGains[1][1] * simplePIDErrorAccumulator[2];
                        Dterm = 0.1*simplePIDGains[1][2] * xErrorDiff.getEntry(2, 0);
                    }
                }

                headingSignal = Pterm + Iterm + Dterm;
                //Log.i("jjb_BEARINGPID", String.format("Bearing PID: total = %.4f  P-term = %.4f  I-term = %.4f  D-term = %.4f, ERROR = %f [deg]",
                        //headingSignal, Pterm, Iterm, Dterm, angleError * 180.0 / Math.PI));

                // Determine which controller to use, simple PID or P-PI pos./vel. cascade
                //if (containers.executingProfile.get() == 1) {
                //    PPICascade();
                //}
                //else {
                simplePID();
                //}
            }
            thrustAndBearingFractionsFromErrorSignal();
        }
        else { // some form of teleoperation is occurring, so don't accumulate error and don't try to control anything
            zeroErrors();
        }

        /*
        //generate MOTOR sensor data from thrust fraction
        double expectedSpeed = velocityMotorMap.thrustFractionToVelocity(containers.thrustFraction.get());
        RealMatrix z = MatrixUtils.createRealMatrix(1,1);
        z.setEntry(0, 0, expectedSpeed);
        RealMatrix R = MatrixUtils.createRealMatrix(1,1);
        R.setEntry(0, 0, 0.0);
        Datum datum = new Datum(SENSOR_TYPE.MOTOR,System.currentTimeMillis(),z,R,(int)containers.self.id.get());
        datumListener.newDatum(datum);
        */

        motorCommandsFromThrustAndBearingFractions();
    }

    void simplePID() {
        // Operate on x,y, and theta concurrently.
        // The boat's heading should converge to the direction of water flow (i.e. fx,fy)
        // That way the boat just needs to go straight forward to stay on the right spot

        double srssP = SRSS(xError.getEntry(0,0),xError.getEntry(1,0));
        double srssI = SRSS(simplePIDErrorAccumulator[0],simplePIDErrorAccumulator[1]);
        double srssD = SRSS(xErrorDiff.getEntry(0,0),xErrorDiff.getEntry(1,0));
        thrustSignal = simplePIDGains[0][0]*srssP + simplePIDGains[0][1]*srssI + simplePIDGains[0][2]*srssD;
    }

    double SRSS(double a, double b) {
        return Math.pow(Math.pow(a,2.0)+Math.pow(b,2.0),0.5);
    }

    void PPICascade() {
        // Operate in two phases
        // If the theta error is above some threshold, focus purely on that, ignoring the velocity profile
        // If the theta error is below that threshold, execute the P-PI cascade on the velocity profile
        // As long as the theta error remains below that threshold, you assume the controller is solving
        //   a 1-D problem, modulating the velocity independently of a simple PID that is correcting theta error

        // Determine which phase you are in based on current theta error
        // PID for heading is always occurring
        double vd; // the desired velocity
        double dd; // the desired distance from target

        //if (Math.abs(xError.getEntry(2, 0)) < headingErrorThreshold) {
        if (!pointing) {
            if (!t0set) {
                t0 = t.doubleValue()/1000.0;
                t0set = true;
            }
            double tRelative = t.doubleValue()/1000.0 - t0;
            // linear interpolate desired velocity
            vd = RMO.interpolate1D(profile,tRelative,1);
            double v = containers.velocityTowardGoal();
            dd = RMO.interpolate1D(profile,tRelative,2);
            double d = containers.distToDest.get();

            //Log.i("jjb_PPI",String.format("PPI: vd = %.3f  actual v = %.3f  fx = %.3f  fy = %.3f  dd = %.3f  actual d = %.3f",vd,v,x.getEntry(5,0),x.getEntry(6,0),dd,d));

            // use actual velocity towards goal (use velocityTowardGoal()), actual distance from goal (distToDest container) to generate errors
            double vError = vd - v;
            double dError = d - dd; // note how this is backwards because we want positive motor action to REDUCE the distance
            PPIErrorAccumulator += PPIGains[0]*dError + vError;
            thrustSignal = PPIGains[1]*(PPIGains[0]*dError + vError) + PPIGains[2]*PPIErrorAccumulator;
        }
        else {
            //TODO: perhaps make this exponentially decay each iteration instead?
            thrustSignal = 0.0;
            //thrustSignal *= 0.5;
            //containers.teleopThrustFraction.set(0.5*containers.teleopThrustFraction.get());
        }
    }

    void motorCommandsFromThrustAndBearingFractions() {
        double m0 = 0.0;
        double m1 = 0.0;
        double T = containers.thrustFraction.get();
        double B = containers.bearingFraction.get();
        double trueT = 0;
        double trueB = 0;
        //RealMatrix z = MatrixUtils.createRealMatrix(1,1);
        //RealMatrix R = MatrixUtils.createRealMatrix(1,1);
        t = System.currentTimeMillis();
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            m0 = clip(T + B, -1, 1);
            m1 = clip(T - B, -1, 1);
            trueT = (m0 + m1)/2;
            trueB = (m0 - m1)/2;
        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            m0 = Math.sqrt(Math.pow(T,2.0)+Math.pow(B,2.0));
            m1 = -2/Math.PI*Math.asin(B/Math.sqrt(Math.pow(T,2.0)+Math.pow(B,2.0)));
            trueT = T;
            trueB = B;
        }
        containers.thrustFraction.set(trueT);
        containers.bearingFraction.set(trueB);

        containers.motorCommands.set(0,m0);
        containers.motorCommands.set(1,m1);

        String velocityMapTestString = String.format("t = %d   X = %.2f   Y = %.2f   trueT = %.4f   trueB = %.4f  m0 = %.3f  m1 = %.3f",
                System.currentTimeMillis(),x.getEntry(0,0),x.getEntry(1,0),trueT,trueB,m0,m1);
        //Log.i("jjb_VEL", velocityMapTestString);
    }

    void thrustAndBearingFractionsFromErrorSignal() {
        double angleError = xError.getEntry(2,0);
        double T = 0;
        double B = 0;
        double clippedAngleError = clip(Math.abs(angleError), 0, headingErrorThreshold);
        double thrustReductionRatio = Math.cos((Math.PI/2.0)/headingErrorThreshold*clippedAngleError);
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            double maxThrust = thrustReductionRatio*SAFE_DIFFERENTIAL_THRUST;
            T = clip(thrustSignal,-maxThrust,maxThrust);
            if (Math.signum(-headingSignal) < 0) {
                B = clip(-headingSignal,Math.signum(-headingSignal)*MAX_DIFFERENTIAL_BEARING,Math.signum(-headingSignal)*MIN_DIFFERENTIAL_BEARING);
            }
            else {
                B = clip(-headingSignal,MIN_DIFFERENTIAL_BEARING,MAX_DIFFERENTIAL_BEARING);
            }

            //Log.i("jjb_TANDB", String.format("clippedAngleError = %.3f  thrustReductionRatio = %.3f  T = %.3f  B = %.3f", clippedAngleError, thrustReductionRatio,T,B));

        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            T = clip(thrustSignal,0,SAFE_VECTORED_THRUST);
            B = clip(-headingSignal,-1,1);
        }
        containers.thrustFraction.set(T);
        containers.bearingFraction.set(B);
    }

    void updateFromKnowledgeBase() {
        // update gains
        for (int i = 0; i < 3; i++) {
            simplePIDGains[0][i] = containers.thrustPIDGains.get(i);
            simplePIDGains[1][i] = containers.bearingPIDGains.get(i);
            PPIGains[i] = containers.thrustPPIGains.get(i);
        }

        // remember to subtract device.{.id}.home from the destination so xd is centered about (0,0) like x
        xd = containers.NDV_to_RM(containers.self.device.dest).subtract(containers.NDV_to_RM(containers.self.device.home));

        // update current state
        for (int i = 0; i < stateSize; i++) {
            x.setEntry(i,0,containers.localState.get(i));
        }

       //Log.i("jjb_XD","xd = " + RMO.realMatrixToString(xd));
       //Log.i("jjb_X","x = " + RMO.realMatrixToString(x));
    }

    public void newProfile(RealMatrix profile) {
        this.profile = profile;
        containers.executingProfile.set(1);
        simplePIDErrorAccumulator = new double[]{0,0,0}; // set basic PID error accumulators to zero
        t0set = false;
    }

    public void shutdown() {
    }

    double clip(double value,double min, double max) {
        double result = value;
        if (value > max) { result = max; }
        if (value < min) { result =  min; }
        return result;
    }

    double map(double input, double input_min, double input_max,
                             double output_min, double output_max) {
        return (input - input_min) / (input_max - input_min)
                * (output_max - output_min) + output_min;
    }



}
