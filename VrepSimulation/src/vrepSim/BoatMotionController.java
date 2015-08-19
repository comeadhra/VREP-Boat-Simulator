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
    boolean t0set;
    LutraMadaraContainers containers;
    final double headingErrorThreshold = 15.0*Math.PI/180.0; // +/- 15 deg
    double simplePIDGains[][];
    double PPIGains[];
    double PPIErrorAccumulator; // [Pos-P*(pos error) + vel error] accumulation
    double[] simplePIDErrorAccumulator; // cols: x,y,th
    public static final double SAFE_DIFFERENTIAL_THRUST = 0.6;
    public static final double MIN_DIFFERENTIAL_BEARING = 0.15;
    public static final double MAX_DIFFERENTIAL_BEARING = 0.4;

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
        xError = MatrixUtils.createRealMatrix(4,1); // [x y th v]
        xErrorOld = MatrixUtils.createRealMatrix(4,1); // [x y th v]
        xd = MatrixUtils.createRealMatrix(2,1); // [x y]
        PPIErrorAccumulator = 0;
        simplePIDErrorAccumulator = new double[]{0,0,0};
        t = System.currentTimeMillis();
        datumListener = boatEKF;
        velocityMotorMap = new VelocityMotorMap(containers);
        PPIGains = new double[3];
        simplePIDGains = new double[2][3];
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
            System.out.println(angleErrorString);

            xError.setEntry(2, 0, angleError);

            // if you are within a sufficient distance from the goal, stop following velocity profiles
            containers.distToDest.set(RMO.distance(x.getSubMatrix(0, 1, 0, 0), xd.getSubMatrix(0, 1, 0, 0)));
            if (containers.distToDest.get() < containers.sufficientProximity.get()) {
                containers.executingProfile.set(0);
                // reset PPICascade accumulated error variables
                PPIErrorAccumulator = 0;
            }

            // Heading PID control is always operating!
            for (int i = 0; i < 3; i++) {
                simplePIDErrorAccumulator[i] += xError.getEntry(i,0)*dt;
            }
            xErrorDiff = xError.subtract(xErrorOld).scalarMultiply(1.0/dt);

            String xErrorDiffString = String.format("xError = %s",RMO.realMatrixToString(xErrorDiff));
            //System.out.println(xErrorDiffString);

            headingSignal = simplePIDGains[1][0]*xError.getEntry(2,0) + // P
                                        simplePIDGains[1][1]*simplePIDErrorAccumulator[2] + // I
                                            simplePIDGains[1][2]*xErrorDiff.getEntry(2,0); // D

            // Determine which controller to use, simple PID or P-PI pos./vel. cascade
            //if (containers.executingProfile.get() == 1) {
            //    PPICascade();
            //}
            //else
            if (Math.abs(angleError) < headingErrorThreshold) {
                simplePID();
            }
            else {
                thrustSignal *= 0.5;
            }
            thrustAndBearingFractionsFromErrorSignal();
        }
        else { // some form of teleoperation is occurring, so don't accumulate error and don't try to control anything
            zeroErrors();
        }

        //generate MOTOR sensor data from thrust fraction
        double expectedSpeed = velocityMotorMap.thrustFractionToVelocity(containers.thrustFraction.get());
        RealMatrix z = MatrixUtils.createRealMatrix(1,1);
        z.setEntry(0, 0, expectedSpeed);
        RealMatrix R = MatrixUtils.createRealMatrix(1,1);
        R.setEntry(0, 0, 0.0);
        Datum datum = new Datum(SENSOR_TYPE.MOTOR,System.currentTimeMillis(),z,R,(int)containers.self.id.get());
        datumListener.newDatum(datum);

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

        if (Math.abs(xError.getEntry(2, 0)) < headingErrorThreshold) {
            if (!t0set) {
                t0 = t.doubleValue()/1000.0;
                t0set = true;
            }
            double tRelative = t.doubleValue()/1000.0 - t0;
            // linear interpolate desired velocity
            vd = RMO.interpolate1D(profile,tRelative,1);
            dd = RMO.interpolate1D(profile,tRelative,2);

            // use actual velocity towards goal (use velocityTowardGoal()), actual distance from goal (distToDest container) to generate errors
            double vError = vd - containers.velocityTowardGoal();
            double dError = dd - containers.distToDest.get();
            PPIErrorAccumulator += PPIGains[0]*dError + vError;
            thrustSignal = PPIGains[1]*(PPIGains[0]*dError + vError) + PPIGains[2]*PPIErrorAccumulator;
        }
        else {
            //TODO: perhaps make this exponentially decay each iteration instead?
            thrustSignal *= 0.5;
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
        RealMatrix z = MatrixUtils.createRealMatrix(1,1);
        RealMatrix R = MatrixUtils.createRealMatrix(1,1);
        t = System.currentTimeMillis();
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            m0 = clip(T + B, -1, 1);
            m1 = clip(T - B, -1, 1);
            trueT = (m0 + m1)/2;
            trueB = (m0 - m1)/2;
        }
        if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            m0 = Math.sqrt(Math.pow(T,2.0)+Math.pow(B,2.0));
            m1 = 2/Math.PI*Math.asin(B/Math.sqrt(Math.pow(T,2.0)+Math.pow(B,2.0)));
            trueT = T;
            trueB = B;
        }
        containers.thrustFraction.set(trueT);
        containers.bearingFraction.set(trueB);

        containers.motorCommands.set(0,m0);
        containers.motorCommands.set(1,m1);

        String velocityMapTestString = String.format("t = %d   X = %.2f   Y = %.2f   trueT = %.4f   trueB = %.4f",
                System.currentTimeMillis(),x.getEntry(0,0),x.getEntry(1,0),trueT,trueB);
        System.out.println(velocityMapTestString);
    }

    void thrustAndBearingFractionsFromErrorSignal() {
        double angleError = xError.getEntry(2,0);
        double T = 0;
        double B = 0;
        double angleErrorRatio = Math.abs(angleError)/Math.PI;
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            double Bmax = MIN_DIFFERENTIAL_BEARING + angleErrorRatio*(MAX_DIFFERENTIAL_BEARING - MIN_DIFFERENTIAL_BEARING);
            double Tmax = (1 - angleErrorRatio)*SAFE_DIFFERENTIAL_THRUST;
            T = clip(thrustSignal,-Tmax,Tmax);
            B = clip(headingSignal,-Bmax,Bmax);
        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
            T = clip(thrustSignal,0,SAFE_VECTORED_THRUST);
            B = clip(headingSignal,-1,1);
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

        //Log.w("jjb","xd = " + RMO.realMatrixToString(xd));
        //Log.w("jjb","x = " + RMO.realMatrixToString(x));
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
