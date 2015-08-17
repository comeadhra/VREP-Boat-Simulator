package vrepSim;

import com.gams.utility.GpsPosition;
import com.madara.threads.BaseThread;
import com.madara.threads.Threader;

import coppelia.remoteApi;
import coppelia.FloatWA;
import coppelia.IntW;


public class vrepSimBoat {

    private static remoteApi vrep;

    //VREP boat simulation model
    private final String MODEL_FILE = System.getenv("GAMS_ROOT")
            + "/resources/vrep/boat_ext.ttm";
    private final int nodeInd;           //index of node
    private final int modelId;           //model id
    private final int clientId;          //id of vrep simulation scene
    private final String rightMotorSig = "lm";
    private final String leftMotorSig = "rm";
    private final String vrepHost = "127.0.0.1";
    private final int vrepPort;          //vrep communication port 
    private final GpsPosition  swPosition;        //GPS coordinates of origin
    private float lm = 0;
    private float rm = 0;
    private GpsPosition gps;       //current gps reading
    private double[] gyro;         //current gyro reading
    private double[] compass;      //current compass reading
    private Threader threader;
    public vrepSimBoat(int nodeInd, GpsPosition initGps, 
                                    GpsPosition swPosition){
    //Launch Boat controller
        //TODO
        //Instantiate Threader for thread handling
        threader = new Threader();
        //Launch test controller
        threader.run("5.0", new ControllerTestThread());
        
        //Initialise VREP remote API
        vrep = new remoteApi();

        //Store simulation properties
        this.nodeInd = nodeInd;
        vrepPort = 19905 + this.nodeInd + 1;
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
        vrep.simxLoadModel(clientId, MODEL_FILE, 0, modelIdW,
                remoteApi.simx_opmode_oneshot_wait);
        
        //Extract model id from wrapper
        modelId = modelIdW.getValue();
        
        //Set model initial position 
        FloatWA initPos = gpsToVrep(gps);
        vrep.simxSetObjectPosition(clientId, modelId, remoteApi.sim_handle_parent,
                initPos, remoteApi.simx_opmode_oneshot);
        
        
        //Start sensor threads
        threader.run("5.0", new GpsThread());
        threader.run("5.0", new CompassThread());
        threader.run("5.0", new GyroThread());
        threader.run("100.0", new MotorUpdateThread());

    }
    //Thread to get GPS position from VREP, add noise
    //and pass into the data back to the controlling
    //algorithm

    private class GpsThread extends BaseThread {

        @Override
        public void run() {
            //Get absolute GPS position
            gps = getPosition();
            //Add noise to GPS 

            //Set datum listener with new value
        }
    }

    private class GyroThread extends BaseThread {
        
        @Override
        public void run() {
            //Get absolute gyro reading
            gyro = getGyro();
            //Add noise to gyro 

            //Set datum listener with new value
        }

      
    }

    private class CompassThread extends BaseThread {

        @Override
        public void run() {
            //Get absolute compass reading
            compass = getCompass();
            //Add noise to compass

            //Set datum listener with new value
        }

    }

    private class MotorUpdateThread extends BaseThread {

        float leftMotorSpeed;
        float rightMotorSpeed;

        @Override
        public void run() {
            //TODO: Get motor values from LutraGams
            //Get motor speeds from knowledge base
            leftMotorSpeed = lm; //get value from container
            rightMotorSpeed = rm;

            //Set motor speeds in simulator
            vrep.simxSetFloatSignal(clientId, leftMotorSig, leftMotorSpeed,
                    remoteApi.simx_opmode_oneshot);
            vrep.simxSetFloatSignal(clientId, rightMotorSig, rightMotorSpeed,
                    remoteApi.simx_opmode_oneshot);

        }


    }

    /**
     * Return current position of the boat in GPS co-ordinates
     *
     * @return GpsPosition Position of the boat
     */
    public GpsPosition getPosition() {
        FloatWA vrepPos = new FloatWA(3);
        vrep.simxSetObjectPosition(clientId, modelId, -1,
                vrepPos, remoteApi.simx_opmode_oneshot);
        gps = vrepToGps(vrepPos);
        return gps;
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

    private GpsPosition vrepToGps(FloatWA vrepPos) {
        float[] pos = vrepPos.getArray();
        // assume the Earth is a perfect sphere
        final double EARTH_RADIUS = 6371000.0;
        final double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * Math.PI;
        GpsPosition gpsPos = new GpsPosition();
    // convert the latitude/y coordinates
        // VREP uses y for latitude
        gpsPos.setLatitude((360.0 * pos[1] / EARTH_CIRCUMFERENCE)
                + swPosition.getLatitude());

    // assume the meters/degree longitude is constant throughout environment
        // convert the longitude/x coordinates
        // VREP uses x for longitude
        double r_prime = EARTH_RADIUS * Math.cos(Math.toRadians(swPosition.getLatitude()));
        double circumference = 2 * r_prime * Math.PI;
        gpsPos.setLongitude((360.0 * pos[0] / circumference) + swPosition.getLongitude());

        // do nothing to altitude
        gpsPos.setAltitude(pos[2]);
        return gpsPos;
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
        vrepPosArray[3] = gpsPos.getAltitude();

        return new FloatWA(vrepPosArray);
    }
    
    private class ControllerTestThread extends BaseThread{
        
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
}
