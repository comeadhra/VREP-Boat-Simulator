package vrepSim;

import com.gams.utility.GpsPosition;
import com.madara.threads.BaseThread;
import com.madara.threads.Threader;

import coppelia.remoteApi;
import coppelia.FloatWA;
import coppelia.IntW;
import java.util.ArrayList;
import java.util.List;
import javax.measure.unit.NonSI;
import javax.measure.unit.SI;
import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;
import org.apache.commons.math.stat.regression.SimpleRegression;
import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;


public class vrepSimBoat {

    private static remoteApi vrep;

    //VREP boat simulation model
    private final String MODEL_FILE = System.getenv("GAMS_ROOT")
            + "/resources/vrep/boat_ext.ttm";
    private final int nodeInd;           //index of node
    private final int modelId;           //model id
    private final int clientId;          //id of vrep simulation scene
    private final String leftMotorSig = "motor1";
    private final String rightMotorSig = "motor0";
    private final String vrepHost = "127.0.0.1";
    private final int vrepPort;          //vrep communication port 
    private final GpsPosition  swPosition;        //GPS coordinates of origin
    private GpsPosition gps;       //current gps reading
    private double[] gyro;         //current gyro reading
    private double[] compass;      //current compass reading
    private final Threader threader;
    private final LutraGAMS lutra;
    private final DatumListener datumListener;
    List<Datum> gpsHistory = new ArrayList<>(); // maintain a list of GPS data within some time window
    final double gpsHistoryTimeWindow = 3.0; // if a gps point is older than X seconds, abandon it
    double eBoardGPSTimestamp = 0.0;
    SimpleRegression regX = new SimpleRegression();
    SimpleRegression regY = new SimpleRegression();    
    
    public vrepSimBoat(int nodeInd, GpsPosition initGps, GpsPosition swPosition) {
        
        // start the software that would be running on the boat
        lutra = new LutraGAMS(nodeInd, 1, THRUST_TYPES.DIFFERENTIAL);
        lutra.start(lutra);
        datumListener = lutra.platform.boatEKF;
        
        //Instantiate Threader for thread handling
        threader = new Threader();
        
        //Initialise VREP remote API
        vrep = new remoteApi();

        //Store simulation properties
        this.nodeInd = nodeInd;
        vrepPort = 19905 + this.nodeInd + 1; // each boat on its own port
        this.swPosition = swPosition;
        
        //Open connection to vrep
        clientId = vrep.simxStart(vrepHost, vrepPort, true, true, 2000, 5);

        //Store initial system state
        gps = initGps;
        gyro = new double[3];
        compass = new double[3];
        
        //wrapped modelId 
        IntW modelIdW = new IntW(1);
        
        //Add model to vrep scene
        int returnVal = vrep.simxLoadModel(clientId, MODEL_FILE, 0, modelIdW,
                remoteApi.simx_opmode_oneshot_wait);
        if( returnVal != remoteApi.simx_return_ok )
        {
            System.out.println("Boat Model" + nodeInd + " failed to load.");
            System.exit(-1);
        }
        //Extract model id from wrapper
        modelId = modelIdW.getValue();
        
        //Set model initial position 
        FloatWA initPos = gpsToVrep(gps);
        vrep.simxSetObjectPosition(clientId, modelId, remoteApi.sim_handle_parent,
                initPos, remoteApi.simx_opmode_oneshot);
        
        
        //Start sensor threads
        threader.run(5.0, "gpsThread", new GpsThread());
        threader.run(5.0, "compassThread", new CompassThread());
        threader.run(5.0, "gyroThread", new GyroThread());
        threader.run(5.0, "motorUpdateThread", new MotorUpdateThread());

    }
    //Thread to get GPS position from VREP, add noise
    //and pass into the data back to the controlling
    //algorithm

    private class GpsThread extends BaseThread {

        @Override
        public void run() {
            //Get absolute GPS position
            getPosition();
            double lat = gps.getLatitude();
            double lon = gps.getLongitude();
            
            if (Math.abs(lat) > 1e8 || Math.abs(lon) > 1e8) {
                return;
            }
            
            // TODO: Add noise to GPS             /////////////////////////////////////////////////////////////////////////////////
            
            //Set datum listener with new value
            UTM utmLoc = UTM.latLongToUtm(LatLong.valueOf(lat,lon, NonSI.DEGREE_ANGLE),ReferenceEllipsoid.WGS84);
            if (lutra != null) {
                lutra.platform.containers.longitudeZone.set((long)utmLoc.longitudeZone());
                lutra.platform.containers.latitudeZone.set(java.lang.String.format("%c",utmLoc.latitudeZone()));
            }
            RealMatrix z = MatrixUtils.createRealMatrix(2,1);
            z.setEntry(0, 0, utmLoc.eastingValue(SI.METER));
            z.setEntry(1,0,utmLoc.northingValue(SI.METER));
            RealMatrix R = MatrixUtils.createRealMatrix(2,2);
            R.setEntry(0, 0, 5.0);
            R.setEntry(1,1,5.0);
            Datum datum = new Datum(SENSOR_TYPE.GPS,System.currentTimeMillis(),z,R, nodeInd);
            datumListener.newDatum(datum);

            gpsVelocity(datum);
        }
    }
    
    private void gpsVelocity(Datum datum) {
        Long t = System.currentTimeMillis();

        gpsHistory.add(datum);

        for (int i = gpsHistory.size()-1; i > -1; i--) {
            if ((t.doubleValue() - gpsHistory.get(i).getTimestamp().doubleValue())/1000.0 > gpsHistoryTimeWindow) {
                gpsHistory.remove(i);
            }
        }
        String gpsHistoryString = String.format("There are %d GPS measurements in the history",gpsHistory.size());
        //System.out.println(gpsHistoryString);

        if (gpsHistory.size() < 6) {return;} // need at least 6 data points

        // Least squares linear regression with respect to time
        double[][] xvst = new double[gpsHistory.size()][2];
        double [][] yvst = new double [gpsHistory.size()][2];
        for (int i = 0; i < gpsHistory.size(); i++) {
            double local_t = gpsHistory.get(i).getTimestamp().doubleValue()/1000.0;
            xvst[i][0] = local_t;
            yvst[i][0] = local_t;
            xvst[i][1] = gpsHistory.get(i).getZ().getEntry(0,0);
            yvst[i][1] = gpsHistory.get(i).getZ().getEntry(1,0);
        }
        regX.addData(xvst);
        regY.addData(yvst);
        double xdot = regX.getSlope();
        double ydot = regY.getSlope();

        RealMatrix z = MatrixUtils.createRealMatrix(2,1);
        z.setEntry(0,0,xdot);
        z.setEntry(1, 0, ydot);
        RealMatrix R = MatrixUtils.createRealMatrix(2,2);
        R.setEntry(0, 0, 10.0); // 5 meteres is 2 standard deviations, so 2*sig = 5 --> sig^2 = (5/2)^2 = 6.25
        R.setEntry(1, 1, 10.0);
        Datum datum2 = new Datum(SENSOR_TYPE.DGPS,t,z,R,nodeInd);
        datumListener.newDatum(datum2);        
    }

    private class GyroThread extends BaseThread {
        
        @Override
        public void run() {
            //Get absolute gyro reading
            gyro = getGyro();
            
            //TODO: Add noise to gyro        /////////////////////////////////////////////////////////////////////////////////////
            
            
            if (Math.abs(gyro[2]) > 1000.0) {return;} // need this because once in a while the gyro explodes to some ridiculous number, even if the boat is just sitting still
                        
            RealMatrix z = MatrixUtils.createRealMatrix(1,1);
            z.setEntry(0,0,gyro[2]);
            RealMatrix R = MatrixUtils.createRealMatrix(1,1);
            R.setEntry(0, 0, 0.0004*0.0004); // the noise floor with zero input --> TINY error, so this is supreme overconfidence
            Datum datum = new Datum(SENSOR_TYPE.GYRO,System.currentTimeMillis(),z,R,nodeInd);
            datumListener.newDatum(datum);            
        }

      
    }

    private class CompassThread extends BaseThread {

        @Override
        public void run() {
            //Get absolute compass reading
            compass = getCompass();            
            //TODO: Add noise to compass   /////////////////////////////////////////////////////////////////////////////////////////
            
            double yaw = compass[2];
            
            if (Math.abs(yaw) > 1000.0) { // protect against weird explosions in value
                return;
            }
            
            // alter the measurement by 2*PI until its difference with current yaw is minimized
            if (lutra.platform.containers != null) {
                double currentYaw = lutra.platform.containers.eastingNorthingBearing.get(2);
                double angleBetween = currentYaw - yaw;
                double originalSign = Math.signum(angleBetween);
                double newYawMeasurement = yaw;
                while (true) {
                    newYawMeasurement = yaw + originalSign * 2 * Math.PI;
                    double newAngleBetween = currentYaw - newYawMeasurement;
                    if (Math.abs(newAngleBetween) < Math.abs(angleBetween)) {
                        angleBetween = newAngleBetween;
                        yaw = newYawMeasurement;
                    } else {
                        break;
                    }
                }
            }
            
            RealMatrix z = MatrixUtils.createRealMatrix(1,1);
            z.setEntry(0,0,yaw);
            RealMatrix R = MatrixUtils.createRealMatrix(1,1);
            R.setEntry(0, 0, Math.pow((Math.PI/18.0)/2.0,2.0)); // estimate 10 degrees is 2 std. dev's
            Datum datum = new Datum(SENSOR_TYPE.COMPASS,System.currentTimeMillis(),z,R,nodeInd);
            datumListener.newDatum(datum);
        }

    }

    private class MotorUpdateThread extends BaseThread {

        float leftMotorSpeed;
        float rightMotorSpeed;

        @Override
        public void run() {
            if (lutra != null) {
                //TODO: Get motor values from LutraGams
                //Get motor speeds from knowledge base
                leftMotorSpeed = (float)lutra.platform.containers.motorCommands.get(0); //get value from container
                rightMotorSpeed = (float)lutra.platform.containers.motorCommands.get(1); //get value from container
                
                //System.out.println(String.format("motorCommands.get(0) = %f,  motorCommands.get(1) = %f",leftMotorSpeed,rightMotorSpeed));

                //Set motor speeds in simulator
                vrep.simxSetFloatSignal(clientId, leftMotorSig, leftMotorSpeed,
                        remoteApi.simx_opmode_oneshot);
                vrep.simxSetFloatSignal(clientId, rightMotorSig, rightMotorSpeed,
                        remoteApi.simx_opmode_oneshot);
            }

        }


    }

    public double[] getGyro() {
        FloatWA angv = new FloatWA(3);
        vrep.simxGetObjectVelocity(clientId, modelId, null, angv,
                remoteApi.simx_opmode_oneshot_wait);
        float[] angvArray = angv.getArray();
        gyro = new double[3];

        gyro[0] = angvArray[0];
        gyro[1] = angvArray[1];
        gyro[2] = angvArray[2];

        return gyro;
    }

    public double[] getCompass() {
        
        FloatWA orientation = new FloatWA(3);
        vrep.simxGetObjectOrientation(clientId, modelId, 
                remoteApi.sim_handle_parent, orientation, 
                remoteApi.simx_opmode_oneshot_wait);
        
        float[] arrayOrientation = orientation.getArray();
        compass = new double[3];

        compass[0] = arrayOrientation[0];
        compass[1] = arrayOrientation[1];
        compass[2] = arrayOrientation[2];

        return compass;
    }

    /**
     * Return current position of the boat in GPS co-ordinates
     *
     * @return GpsPosition Position of the boat
     */
    public GpsPosition getPosition() {
        FloatWA vrepPos = new FloatWA(3);
        vrep.simxGetObjectPosition(clientId, modelId, -1,
                vrepPos, remoteApi.simx_opmode_oneshot_wait);
        
        vrepToGps(vrepPos);
        
        //System.out.println(String.format("vrep generated gps LAT = %f, LON = %f",gps.getLatitude(), gps.getLongitude()));
        
        return gps;
    }    
    
    private void vrepToGps(FloatWA vrepPos) {
        float[] pos = vrepPos.getArray();
        // assume the Earth is a perfect sphere
        final double EARTH_RADIUS = 6371000.0;
        final double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * Math.PI;
        // convert the latitude/y coordinates
        // VREP uses y for latitude
        gps.setLatitude((360.0 * pos[1] / EARTH_CIRCUMFERENCE) + swPosition.getLatitude());

        // assume the meters/degree longitude is constant throughout environment
        // convert the longitude/x coordinates
        // VREP uses x for longitude
        double r_prime = EARTH_RADIUS * Math.cos(Math.toRadians(swPosition.getLatitude()));
        double circumference = 2 * r_prime * Math.PI;
        gps.setLongitude((360.0 * pos[0] / circumference) + swPosition.getLongitude());

        // do nothing to altitude
        gps.setAltitude(pos[2]);
    }

    private FloatWA gpsToVrep(GpsPosition gpsPos) {
       
        double [] vrepPosArray = new double[3];
        
        // assume the Earth is a perfect sphere
        final double EARTH_RADIUS = 6371000.0;
        final double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * Math.PI;

        // convert the latitude/y coordinates
        vrepPosArray[1] = (gpsPos.getLatitude() - swPosition.getLatitude()) / 360.0 * EARTH_CIRCUMFERENCE;

    // assume the meters/degree longitude is constant throughout environment
        // convert the longitude/x coordinates
        double r_prime = EARTH_RADIUS * Math.cos(Math.toRadians(swPosition.getLatitude()));
        double circumference = 2 * r_prime * Math.PI;
        vrepPosArray[0] = (gpsPos.getLongitude() - swPosition.getLongitude()) / 360.0 * circumference;

        // do nothing to altitude
        vrepPosArray[2] = gpsPos.getAltitude();

        return new FloatWA(vrepPosArray);
    }
    
    /*
    private class ControllerTestThread extends BaseThread {
        
        public void run()
        {
             
             System.out.println("B" + nodeInd + ": GPS: " + gps.getLatitude() 
                                              + " , " + gps.getLongitude() );
             System.out.println("B" + nodeInd + ": Yaw:" + gyro[3]);
             System.out.println("B" + nodeInd + ": Heading:" + compass[3]);
             
             //If user input is available
             //int id = ;
             //if(id == nodeInd){
                //rm =;
                //lm =;
            
        }    
        
    }
    */
}
