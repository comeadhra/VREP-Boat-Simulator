package vrepSim;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;
import org.jscience.geography.coordinates.LatLong;

import com.madara.KnowledgeBase;

import javax.measure.unit.NonSI;


/**
 * @author jjb
 */
public class BoatEKF implements DatumListener {

    // Things that update explicitly via Listener
    RealMatrix z; // new measurement to include
    RealMatrix H; // dz/dx associated with z
    RealMatrix R; // covariance associated with z

    public final int stateSize = 6; // state is global except for expected-motor velocities
    // state = [x y theta w v fx fy] OR [x y theta w vx vy]

    double[] initial_x = new double[stateSize];
    RealMatrix x = MatrixUtils.createColumnRealMatrix(initial_x); // state
    RealMatrix initialP = MatrixUtils.createRealIdentityMatrix(stateSize);
    RealMatrix P; // stateCov
    RealMatrix K; // Kalman gain
    RealMatrix Ktemp; // a placeholder during a calculation with several RealMatrix objects
    RealMatrix QBase = MatrixUtils.createRealIdentityMatrix(stateSize); // growth of uncertainty with time (before time step modification)
    RealMatrix Q; // growth of uncertainty with time (after time step modification)
    RealMatrix G = MatrixUtils.createRealIdentityMatrix(stateSize); // coordinate transformation of uncertainty
    RealMatrix Phi = MatrixUtils.createRealIdentityMatrix(stateSize); // state transition (x_{k+1} = Phi*x_{k})
    RealMatrix Phi_k =  MatrixUtils.createRealIdentityMatrix(stateSize); // (I + F*dt), propagation of uncertainty (P_{k+1} = Phi_k*P_{k}*Phi_k' + GQG')
    RealMatrix dz; // dz = z - h(x)... in a linear KF, h(x) = Hx
    RealMatrix S; // the innovation
    RealMatrix dtemp; // a placeholder during a calculation with several RealMatrix objects
    RealMatrix home_RM; // used to create local X,Y

    Long t; // current time
    final double ROLLBACK_LIMIT = 1.0; // seconds allowed for a rollback before measurements are just abandoned

    KnowledgeBase knowledge;
    LutraMadaraContainers containers;

    public BoatEKF(KnowledgeBase knowledge,LutraMadaraContainers containers) {
        t = java.lang.System.currentTimeMillis();
        this.knowledge = knowledge;
        this.containers = containers;
        containers.localState.resize(stateSize);

        // default Q and P (use other constructor to override these)
        //QBase = QBase.scalarMultiply(0.1);
        //QBase.setEntry(0,0,1.0);
        //QBase.setEntry(1,1,1.0);
        //initialP = initialP.scalarMultiply(0.1);
        initialP.setEntry(0,0,5.0); // default GPS covariance is much larger than other parts of state
        initialP.setEntry(1,1,5.0); // default GPS covariance is much larger than other parts of state
        P = initialP.copy();
    }

    public BoatEKF(KnowledgeBase knowledge, LutraMadaraContainers containers, RealMatrix x, RealMatrix P, RealMatrix QBase) {
        this(knowledge,containers);
        this.x = x;
        this.P = P;
        this.QBase = QBase;
    }

    public void shutdown() {
    }

    public synchronized void updateKnowledgeBase() {
        for (int i = 0; i < stateSize; i++) {
            containers.localState.set(i, x.getEntry(i, 0));
        }
        containers.localStateXYCovariance.set(0,P.getEntry(0,0));
        containers.localStateXYCovariance.set(1,P.getEntry(1,0));
        containers.localStateXYCovariance.set(2,P.getEntry(0,1));
        containers.localStateXYCovariance.set(3, P.getEntry(1, 1));

        //Log.i("jjb_EKF", String.format("x = %s", RMO.realMatrixToString(x)));
        //Log.i("jjb_EKF", String.format("P = %s", RMO.realMatrixToString(P)));
    }

    public synchronized void resetLocalization() { // if home is reset, we may want to reset local state as well
        containers.compassInitialized.set(0);
        containers.gpsInitialized.set(0);
        containers.localized.set(0);
        for (int i = 0; i < stateSize; i++) {
            containers.localState.set(i, 0.0);
        }
        x = MatrixUtils.createColumnRealMatrix(initial_x);
        P = initialP.copy();
    }

    @Override
    public synchronized void newDatum(Datum datum) {

        //String threadID = String.format(" -- thread # %d",Thread.currentThread().getId());
        //Log.i("jjb_EKF",String.format("received %s datum z = %s", Datum.typeString(datum.getType()), datum.getZ().toString()));
        if (datum.getType() == SENSOR_TYPE.COMPASS) {
            //Log.i("jjb_COMPASS",String.format("received compass datum z = %.1f", datum.getZ().getEntry(0,0)*180.0/Math.PI));
        }
        if (datum.getType() == SENSOR_TYPE.GYRO) {
            //Log.i("jjb_GYRO",String.format("received gyro datum z = %.2f", datum.getZ().getEntry(0,0)*180.0/Math.PI));
        }
        if (datum.getType() == SENSOR_TYPE.GPS) {
            //System.out.println("jjb_GPS" + String.format("received GPS datum X = %.6e  Y = %.6e", datum.getZ().getEntry(0,0), datum.getZ().getEntry(1,0)));
        }

        //String timeString = String.format("EKF.t BEFORE = %d",t);
        //Long old_t = t;

        // update z and R
        if (datum.isType(SENSOR_TYPE.GPS)) { // subtract home so localization is centered around (0,0)
            RealMatrix _z = datum.getZ();
            if (containers.gpsInitialized.get() != 0) { // subtract home ONLY IF gps is already initialized
                home_RM = containers.NDV_to_RM(containers.self.device.home);
                z = _z.subtract(home_RM.getSubMatrix(0,1,0,0));
            }
            else {
                z = _z.copy();
            }
        }
        else {
            z = datum.getZ().copy();
        }

        R = datum.getR().copy();

        // warning if datum timestamp is too far away from filter's current time
        if ((datum.getTimestamp().doubleValue() - t.doubleValue())/1000.0 > ROLLBACK_LIMIT) {
            String warning = String.format(
                    "WARNING: %s sensor is more than %f seconds AHEAD of filter",datum.getType().typeString,ROLLBACK_LIMIT);
            //Log.w("jjb",warning);
        }
        else if ((datum.getTimestamp().doubleValue() - t.doubleValue())/1000.0 < -ROLLBACK_LIMIT) {
            String warning = String.format(
                    "WARNING: %s sensor is more than %f seconds BEHIND of filter",datum.getType().typeString,ROLLBACK_LIMIT);
            //Log.w("jjb",warning);
        }

        // first GPS and compass (i.e. positions) datum are put into state directly
        // velocity parts of state are initialized at zero
        if (containers.gpsInitialized.get() == 0) {
            if (datum.isType(SENSOR_TYPE.GPS)) {
                double[] _z = new double[] {z.getEntry(0,0),z.getEntry(1,0),x.getEntry(2,0)};

                String aaa = String.format("GPS initialization x = %s",RMO.realMatrixToString(MatrixUtils.createColumnRealMatrix(_z)));
                //Log.w("jjb",aaa);

                for (int i = 0; i < 3; i++) {
                    containers.eastingNorthingBearing.set(i,_z[i]);
                    containers.self.device.home.set(i,_z[i]);
                    containers.self.device.dest.set(i,_z[i]);
                    containers.self.device.source.set(i,_z[i]);
                }

                x.setEntry(0,0,0.0);
                x.setEntry(1,0,0.0);
                containers.localState.set(0,0.0);
                containers.localState.set(1,0.0);
                containers.gpsInitialized.set(1);

                LatLong latLong = containers.LocalXYToLatLong();
                containers.self.device.location.set(0,latLong.latitudeValue(NonSI.DEGREE_ANGLE));
                containers.self.device.location.set(1,latLong.longitudeValue(NonSI.DEGREE_ANGLE));
                containers.self.device.location.set(2,0.0);

                if (containers.compassInitialized.get() == 1) {
                    containers.localized.set(1);
                }

                //Log.w("jjb", "GPS is now initialized");
                return;
            }
        }

        if (containers.compassInitialized.get() == 0) { // TODO: is there a way to have the compass update even if you don't have a GPS lock?
            if (datum.isType(SENSOR_TYPE.COMPASS)) {
                double[] x_array = containers.NDV_to_DA(containers.eastingNorthingBearing);
                double[] _z = new double[] {x_array[0],x_array[1],z.getEntry(0,0)};

                String aaa = String.format("Compass initialization x = %s",RMO.realMatrixToString(MatrixUtils.createColumnRealMatrix(_z)));
                //Log.w("jjb",aaa);

                for (int i = 0; i < 3; i++) {
                    containers.eastingNorthingBearing.set(i,_z[i]);
                    containers.self.device.home.set(i,_z[i]);
                    containers.self.device.dest.set(i,_z[i]);
                    containers.self.device.source.set(i,_z[i]);
                }

                x.setEntry(2,0,z.getEntry(0,0));
                containers.localState.set(2,z.getEntry(0,0));
                containers.compassInitialized.set(1);
                if (containers.gpsInitialized.get() == 1) {
                    containers.localized.set(1);
                }

                //Log.w("jjb", "Compass is now initialized");
                return;
            }
        }

        if (containers.localized.get() == 0) {
            timeStep();
            return;
        }

        predict();

        //timeString = timeString + String.format(",  EKF.t AFTER = %d",t);
        //timeString = timeString + String.format(",  datum.t = %d",datum.getTimestamp());
        //timeString = timeString + String.format(",  EKF.dt = %d,  datum LAG = %d",t-old_t,t-datum.getTimestamp());
        //Log.i("jjb_EKF",timeString);

        // given datum.type, construct H
        setH(datum);
        sensorUpdate(datum.getType());

    }



    private synchronized void setH(Datum datum) {
        // x y th w vx vy
        //double s = Math.sin(x.getEntry(2, 0));
        //double c = Math.cos(x.getEntry(2, 0));
        //double vx = x.getEntry(4, 0);
        //double vy = x.getEntry(5, 0);
        //double v = x.getEntry(4, 0);
        //double fx = x.getEntry(5, 0);
        //double fy = x.getEntry(6, 0);
        //double ft = x.getEntry(7,0);

        RealMatrix newH = MatrixUtils.createRealMatrix(z.getRowDimension(),stateSize);
        if (datum.getType() == SENSOR_TYPE.GPS) {
            newH.setEntry(0,0,1.0);
            newH.setEntry(1,1,1.0);
        }
        else if (datum.getType() ==  SENSOR_TYPE.COMPASS) {
            newH.setEntry(0,2,1.0);
        }
        else if (datum.getType() == SENSOR_TYPE.GYRO) {
            newH.setEntry(0,3,1.0);
        }
        /*else if (datum.getType() == SENSOR_TYPES.IMU) {
            newH.setEntry(0,2,fx*s-fy*c);
            newH.setEntry(0,3,1.0);
            newH.setEntry(0,5,-c);
            newH.setEntry(0,6,-s);
            newH.setEntry(1,2,fx*c+fy*s);
            newH.setEntry(1,5,s);
            newH.setEntry(1,6,c);
        }
        */
        /*
        else if (datum.getType() == SENSOR_TYPE.MOTOR) {
            newH.setEntry(0,3,1.0);
            //newH.setEntry(1,4,1.0); // if you had a rotational velocity map
        }
        */
        else if (datum.getType() == SENSOR_TYPE.DGPS) {
            /*
            newH.setEntry(0,2,-v*s);
            newH.setEntry(0,3,c);
            //newH.setEntry(0,5,-1.0);
            newH.setEntry(1,2,v*c);
            newH.setEntry(1,3,s);
            //newH.setEntry(1,6,-1.0);
            */
            newH.setEntry(0,4,1.0);
            newH.setEntry(0,5,1.0);
        }

        this.H = newH;
    }
    public synchronized void sensorUpdate(SENSOR_TYPE type) {
        // compute kalman gain = PH'inv(HPH'+R)
        Ktemp = H.multiply(P).multiply(H.transpose()).add(R);
        Ktemp = RMO.inverse(Ktemp);
        Ktemp = P.multiply(H.transpose()).multiply(Ktemp);
        K = Ktemp.copy();

        //Log.i("jjb_EKF","P = " + P.toString());
        //Log.i("jjb_EKF","H = " + H.toString());
        //Log.i("jjb_EKF","K = " + K.toString());

        // compute innovation (dz), remember in EKF, dz = z - h(x), not z - Hx
        // z - Hx will work for simple measurements, where H is just ones
        // So z - Hx will work for GPS, compass, and gyro. Not sure about IMU just yet.
        // dz = MatrixUtils.createRealMatrix(z.getRowDimension(), 1);
        dz = z.subtract(H.multiply(x));

        // compute innovation covariance S = HPH' + R
        S = H.multiply(P).multiply(H.transpose());

        // check "validation gate" - calculate Mahalanobis distance d = sqrt(dz'*inv(S)*dz)
        dtemp = dz.transpose().multiply(RMO.inverse(S)).multiply(dz);
        double d = Math.sqrt(dtemp.getEntry(0,0));

        boolean incorporate = false;
        if (type == SENSOR_TYPE.GPS) {
            // if Mahalanobis distance is below threshold, update state estimate, x_{k+1} = x_{k} + K*(dz)
            // and update state covariance P_{k+1} = (I - KH)P
            //String a = String.format("Mahalanobis distance = %f",d);
            //Log.w("jjb", a);
            if (d > 3.0) {
                //Log.w("jjb","WARNING, Mahalanobis distance is > 3. Measurement will be ignored.");
            }
            else {
                incorporate = true;
                containers.gpsWatchdog.set(1L);
            }
        }
        else {
            incorporate = true;
        }
        if (incorporate) {
            x = x.add(K.multiply(dz));
            P = (MatrixUtils.createRealIdentityMatrix(P.getRowDimension()).subtract(K.multiply(H))).multiply(P);
        }


        updateKnowledgeBase();
    }

    public synchronized void predict() {

        //String threadID = String.format(" -- thread # %d",Thread.currentThread().getId());
        //Log.w("jjb","BoatEKF.predict()" + threadID);

        double dt = timeStep();
        //String a = String.format("dt = %f",dt);
        //Log.d("jjb",a);

        // update Q with dt
        Q = QBase.scalarMultiply(dt);

        //double s = Math.sin(x.getEntry(2, 0));
        //double c = Math.cos(x.getEntry(2, 0));
        //double w = Math.cos(x.getEntry(3, 0));
        //double vx = x.getEntry(4, 0);
        //double vy = x.getEntry(5,0);
        //double v = x.getEntry(3,0);

        // Update Phi and Phi_k with current state and dt
        Phi.setEntry(0, 4, dt);
        Phi.setEntry(1, 5, dt);
        Phi.setEntry(2, 3, dt);

        Phi_k.setEntry(0, 4, dt);
        Phi_k.setEntry(1, 5, dt);
        Phi_k.setEntry(2, 3, dt);

        // Update G with current state
        //G.setEntry(0,0,c);
        //G.setEntry(0,1,-s);
        //G.setEntry(1,0,s);
        //G.setEntry(1,1,c);

        // Update state and state covariance
        x = Phi.multiply(x);

        // wrap theta
        while (Math.abs(x.getEntry(2,0)) > Math.PI) {
            x.setEntry(2,0,x.getEntry(2,0) - Math.signum(x.getEntry(2,0))*2*Math.PI);
        }

        P = (Phi_k.multiply(P).multiply(Phi_k.transpose())).add(G.multiply(Q).multiply(G.transpose()));

        updateKnowledgeBase();
    }


    public synchronized double timeStep() {
        // Update dt
        Long old_t = t;
        t = System.currentTimeMillis();
        return (t.doubleValue() - old_t.doubleValue())/1000.0;
    }

    /*
    public synchronized boolean rollBack(Long old_t) {
        // roll back the state of the filter
        double secondsBack = (old_t.doubleValue() - t.doubleValue()) / 1000.0;

        // return true if roll back occurred, return false if roll back is too far
        if (secondsBack < ROLLBACK_LIMIT) {
            // do stuff

            return true;
        }
        else {
            return false;
        }
    }
    */

    /*
    synchronized double angleDifference(double a, double b) {
        // subtract sign(a-b)*2*PI until the abs(a-b) starts increasing
        double angleBetween = a - b;
        while (true) {
            double newAngleBetween = angleBetween - Math.signum(angleBetween)*2*Math.PI;
            if (Math.abs(newAngleBetween) < Math.abs(angleBetween)) {
                angleBetween = newAngleBetween;
            }
            else {
                return angleBetween;
            }
        }
    }
    */

}
