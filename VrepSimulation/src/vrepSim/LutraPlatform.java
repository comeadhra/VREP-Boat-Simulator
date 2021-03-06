package vrepSim;

import com.gams.controllers.BaseController;
import com.gams.platforms.BasePlatform;
import com.gams.platforms.PlatformReturnStatusEnum;
import com.gams.platforms.PlatformStatusEnum;
import com.gams.utility.Position;
import com.gams.utility.Axes;
import com.madara.EvalSettings;
import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;
import com.madara.threads.BaseThread;
import com.madara.threads.Threader;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;
import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;

import java.util.Arrays;

import javax.measure.unit.NonSI;

import edu.cmu.ri.crw.data.UtmPose;


/**
 * GAMS BasePlatform implementation
 *
 * @author jjb
 *
 */
public class LutraPlatform extends BasePlatform {

    KnowledgeBase knowledge;

    BoatEKF boatEKF;
    BoatMotionController boatMotionController;
    HysteresisFilter hysteresisFilter;
    Threader threader;
    boolean homeSet = false;
    VelocityProfileListener velocityProfileListener;
    LutraMadaraContainers containers;
    THRUST_TYPES thrustType;
    Long t;
    final double METERS_PER_LATLONG_DEGREE = 111*1000;
    Long startTime;
    Position position;
    double[] currentTarget;
    final int timeSteps = 100;
    RealMatrix velocityProfile = MatrixUtils.createRealMatrix(timeSteps, 3); // t, vel., pos.
    LatLong latLong;
    RealMatrix covariance = MatrixUtils.createRealMatrix(2,2);

    class FilterAndControllerThread extends BaseThread {
        @Override
        public void run() {
            if (containers.resetLocalization.get() == 1) {
                //Log.w("jjb", " !!!!! RESETTING LOCALIZATION !!!!!");
                boatEKF.resetLocalization();
                containers.resetLocalization.set(0);
            }
            else if (containers.localized.get() == 1) {
                if (!homeSet) {
                    homeSet = true;
                    knowledge.print();
                }
                boatEKF.predict();
                boatMotionController.control();
            }
        }
    }

    class KBPrintingThread extends BaseThread {
        @Override
        public void run() {
            knowledge.print();
        }
    }

    class FlowMeasurementThread extends BaseThread {
        @Override
        public void run() {
            if (containers.localized.get() == 1) {
                // need both predicted velocity and global velocity
                // also, only valid when bearing fraction is very small, like < 0.05
                if (containers.bearingFraction.get() < 0.05) {
                    double vx = containers.localState.get(4);
                    double vy = containers.localState.get(5);
                    double vPredicted = boatMotionController.velocityMotorMap.thrustFractionToVelocity(containers.bearingFraction.get());
                    double vxPredicted = vPredicted*Math.cos(containers.localState.get(2));
                    double vyPredicted = vPredicted*Math.sin(containers.localState.get(2));
                    double fx = vx - vxPredicted;
                    double fy = vy - vyPredicted;
                    RealMatrix F = MatrixUtils.createRealMatrix(2,1);
                    F.setEntry(0,0,fx);
                    F.setEntry(1,0,fy);
                    //Position position = new Position(self.device.location.get(0),self.device.location.get(1),0.0);
                    double lat = self.device.location.get(0);
                    double lon = self.device.location.get(1);
                    Datum datum = new Datum(SENSOR_TYPE.FLOW,System.currentTimeMillis(),F,lat,lon,(int)self.id.get());
                    hysteresisFilter.newDatum(datum);
                }
            }
        }
    }

    String ipAddress;

    // Delay for sending modified fields
    EvalSettings evalSettings;

    public LutraPlatform(KnowledgeBase knowledge, THRUST_TYPES thrustType) {
        this.knowledge = knowledge;
        evalSettings = new EvalSettings();
        evalSettings.setDelaySendingModifieds(true);
        threader = new Threader(knowledge);
        this.thrustType = thrustType;
        position = new Position(0.0,0.0,0.0);
        currentTarget = new double[3];
    }

    @Override
    public void init(BaseController controller) {
        super.init(controller);
        containers = new LutraMadaraContainers(knowledge,self,thrustType); // has to occur AFTER super.init, or "self" will be null
        boatEKF = new BoatEKF(knowledge,containers); // has to occur AFTER containers() b/c it needs "self"
        boatMotionController = new BoatMotionController(knowledge,boatEKF,containers);
        hysteresisFilter = new HysteresisFilter(knowledge, containers);
        velocityProfileListener = boatMotionController;
        Datum.setContainersObject(containers);
    }

    public void start() {
        startTime = System.currentTimeMillis();
        threader.run(containers.controlHz, "FilterAndController", new FilterAndControllerThread());
        threader.run(1.0,"KBPrinting", new KBPrintingThread());
        threader.run(SENSOR_TYPE.FLOW.Hz,"FlowMeasurement",new FlowMeasurementThread());
    }


    /**
     * Analyzes the platform.
     *
     * @return status information (@see Status)
     *
     */
    public int analyze() {
        /*
        * Look at distance from current location to desired location.
        * If this distance is above some threshold, call move() to generate velocity profile.
        * Generation of this profile will cause the controller to execute a P-PI cascade.
        * If this distance is below the threshold, do nothing (a simple PID will be running to
        *   automatically enforce station keeping).
        *
        * Platform doesn't get to change its destination. Algorithm or a user does that. Each iteration
        *   in the MAPE loop, the platform just looks where it is and where its destination is.
        */

        t = System.currentTimeMillis();
        /*///////////////////////////////////////////
        if ((containers.localized.get() == 1) && (containers.executingProfile.get() != 1)) {
            double[] goal = new double[]{10, 10};
            moveLocalXY(goal, 1.5);
        }
        *////////////////////////////////////////////
        if ((containers.distToDest.get() > containers.sufficientProximity.get()) &&
                                           (containers.executingProfile.get() != 1)) {

            double finalSpeed = 0.0;
            createProfile(containers.peakVelocity.get(), finalSpeed);
        }

        return PlatformStatusEnum.OK.value();
    }

    /**
     *
     *
     */
    public int rotate(Axes axes) {
        return PlatformStatusEnum.OK.value();
    }

    public double getAccuracy() {
        //return getPositionAccuracy()/METERS_PER_LATLONG_DEGREE; // this is accuracy in latitude and longitude. 0.00001 is about 1 meter.
        return getPositionAccuracy();
    }

    public double getPositionAccuracy() { // not relevant to C++ waypoints algorithm as of 2015-7-26. It uses getAccuracy() instead.
        // TODO: propagate uncertainty in covariance of x,y coordinate to find overall estimate of distance variance in meters
        // TODO: This function returns the MAXIMUM of the sufficient proximity container and that result
        return containers.sufficientProximity.get();
    }

    /**
     * Returns the current GPS position
     *
     */
    public Position getPosition() {
        //Position position = new Position(self.device.location.get(0), self.device.location.get(1), 0.0);
        position.setX(self.device.location.get(0));
        position.setY(self.device.location.get(1));
        position.setZ(0.0);
        return position;
    }

    public int home() {
        return PlatformStatusEnum.OK.value();
    }
    public int land() {
        return PlatformStatusEnum.OK.value();
    }

    void moveLocalXY(double[] localTarget, double proximity) {
        // need to reset motion controller PID error accumulators, start from zero for a new goal
        boatMotionController.zeroErrors();

        // moving in local X,Y requires recreation of Utm global X,Y using current home container Utm

        double[] home = containers.NDV_to_DA(self.device.home);
        self.device.dest.set(0,localTarget[0]+home[0]);
        self.device.dest.set(1, localTarget[1] + home[1]);

    }

    public int move(Position target, double proximity) {
        double[] targetArray;
        targetArray = target.toArray();

        if (Arrays.equals(targetArray, currentTarget)) {

        }
        else {
            currentTarget = targetArray;
            self.device.source.set(0, containers.eastingNorthingBearing.get(0));
            self.device.source.set(1, containers.eastingNorthingBearing.get(1));
            self.device.source.set(2, containers.eastingNorthingBearing.get(2));
        }

        double[] localTarget = containers.PositionToLocalXY(target);
        moveLocalXY(localTarget, proximity);
        if (containers.distToDest.get() < containers.sufficientProximity.get()) {
            return PlatformReturnStatusEnum.PLATFORM_ARRIVED.value();
        }
        else {
            return PlatformReturnStatusEnum.PLATFORM_MOVING.value();
        }
    }

    public int move(UTM utm, double proximity) {
        moveLocalXY(containers.UTMToLocalXY(utm), proximity);
        return PlatformStatusEnum.OK.value();
    }

    public void createProfile(double sustainedSpeed, double finalSpeed) {
        /*
        * createProfile() is called to generate a new velocity profile from current position to a new position.
        */
        double v0 = containers.velocityTowardGoal();
        double vs = sustainedSpeed;
        double vf = finalSpeed;
        double t0 = 0.0; // need the curve to start at relative zero
        double ta = containers.defaultAccelTime;
        double a = (vs-v0)/ta;
        double[] clippedAccel = clipAccel(a,ta);
        a = clippedAccel[0];
        ta = clippedAccel[1];
        double td = containers.defaultDecelTime;
        double d = (vf-vs)/td;
        clippedAccel = clipAccel(d,td);
        d = clippedAccel[0];
        td = clippedAccel[1];
        double L = containers.distToDest.get();
        double ts = 1/vs*(L - 0.5*a*ta*ta - v0*ta - 0.5*d*td*td - vs*td);
        double tf = ta+td+ts;
        /*
        velocityProfile.setEntry(0,0,0);
        velocityProfile.setEntry(0,1,v0);
        velocityProfile.setEntry(1,0,ta);
        velocityProfile.setEntry(1,1,vs);
        velocityProfile.setEntry(2,0,ta+ts);
        velocityProfile.setEntry(2,1,vs);
        velocityProfile.setEntry(3,0,tf);
        velocityProfile.setEntry(3,1,vf);
        */

        RealMatrix timeMatrix = RMO.linspace(t0,tf,timeSteps);
        velocityProfile.setColumn(0, timeMatrix.getColumn(0));
        double dT = L;
        for (int i = 0; i < timeSteps; i++) {
            double T = velocityProfile.getEntry(i,0);
            double vT = 0;
            if (T < ta) { // acceleration period
                vT = v0 + T*a;
            }
            else if ((T > ta) && T < ta+ts) {
                vT = vs;
            }
            else if (T > ta+ts) {
                vT = vs + (T-(ta+ts))*d;
            }
            velocityProfile.setEntry(i, 1, vT);
            if (i == 0) {
                dT = L;
            }
            else if (i == timeSteps-1) {
                dT = 0;
            }
            else {
                dT = dT - (velocityProfile.getEntry(i-1,1)+vT)/2
                        *(velocityProfile.getEntry(i,0)-velocityProfile.getEntry(i-1,0));
            }
            velocityProfile.setEntry(i, 2, dT);
        }

        velocityProfileListener.newProfile(velocityProfile);
    }

    double[] clipAccel(double a, double t) {
        if (Math.abs(a) > containers.maxAccel) { // need lower acceleration, higher time
            // a --> maxAccel, a = maxAccel/a*a
            // ta --> a/maxAccel*ta
            a = containers.maxAccel;
            t = t*containers.maxAccel/a;
        }
        else if (Math.abs(a) < containers.minAccel) { // need higher acceleration, lower time
            a = containers.minAccel;
            t = t*containers.minAccel/a;
        }
        double[] result = new double[]{a,t};
        return result;
    }

    public double getMinSensorRange() {
        return 0.0;
    }

    public double getMoveSpeed() {
        return 10.0;
    }

    public java.lang.String getId() {
        return String.format("%s_%d",getName(),self.id.get());
    }

    public java.lang.String getName() {
        return "Lutra";
    }

    public int sense() {
        containers.connectivityWatchdog.set(1L);

        // move local .x localization state into device.id.location
        // remember to add in device.id.home because .x is about (0,0)
        double[] home = containers.NDV_to_DA(self.device.home);        

        containers.eastingNorthingBearing.set(0,containers.localState.get(0) + home[0]);
        containers.eastingNorthingBearing.set(1,containers.localState.get(1) + home[1]);
        containers.eastingNorthingBearing.set(2,containers.localState.get(2));

        latLong = containers.LocalXYToLatLong();
        self.device.location.set(0,latLong.latitudeValue(NonSI.DEGREE_ANGLE));
        self.device.location.set(1,latLong.longitudeValue(NonSI.DEGREE_ANGLE));
        self.device.location.set(2, 0.0);

        covariance.setEntry(0,0,containers.localStateXYCovariance.get(0));
        covariance.setEntry(1,0,containers.localStateXYCovariance.get(1));
        covariance.setEntry(0,1,containers.localStateXYCovariance.get(2));
        covariance.setEntry(1, 1, containers.localStateXYCovariance.get(3));

        double[] errorEllipse = RMO.covarianceEllipse(covariance);
        containers.errorEllipse.set(0,errorEllipse[0]);
        containers.errorEllipse.set(1,errorEllipse[1]);
        containers.errorEllipse.set(2,errorEllipse[2]);

        //Log.w("jjb","platform sense() KB:");
        //knowledge.print();

        return PlatformStatusEnum.OK.value();
    }

    public void setMoveSpeed(double speed) {
    }

    public int takeoff() {
        return PlatformStatusEnum.OK.value();
    }
    public void stopMove() {
    }

    public void shutdown() {

        boatEKF.shutdown();
        boatMotionController.shutdown();

        // stop threads
        threader.terminate();

        // free any containers
        containers.freeAll();

        // Free threader
        threader.free();
    }
}
