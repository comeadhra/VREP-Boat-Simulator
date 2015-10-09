package vrepSim;

import com.gams.utility.GpsPosition;
import java.util.ArrayList;
import coppelia.remoteApi;
import coppelia.IntW;
import java.util.Random;

public class vrepSimMain {

    public static void main(String[] args) {
        
        //Create remote API object
        remoteApi vrep = new remoteApi();

        //Number of boats in the scene
        final int NUMBER_OF_SIMULATED_BOATS = 1;
        //Initial positions of boats
        Random randomGenerator = new Random();
        GpsPosition[] initPositions = new GpsPosition[NUMBER_OF_SIMULATED_BOATS];
        initPositions[0] = new GpsPosition(40.436902,-79.948678, 0);
        for (int i = 1; i < NUMBER_OF_SIMULATED_BOATS; i++) {
            initPositions[i] = new GpsPosition(40.436902 + (3+randomGenerator.nextDouble()*7.0)*1.0e-5,
                                              -79.948678 + (3+randomGenerator.nextDouble()*7.0)*1.0e-5, 0);
        }

        //Vrep communication settings
        String vrepHost = "127.0.0.1";
        int vrepPort = 19905;

        //GPS co-ordinates of VREP origin, currently set to Panther Hollow
        double sw_lat = 40.436902;
        double sw_long = -79.948678;

        GpsPosition swPosition = new GpsPosition(sw_lat, sw_long, 0.0);

        //Open vrep connection
        System.out.print("connecting to vrep...");
        int clientId
                = vrep.simxStart(vrepHost, vrepPort, true, true, 2000, 5);
        if (clientId == -1) {
            System.out.println("failure connecting to vrep");
            System.exit(-1);
        }

        //load simultion scene
        vrep.simxStopSimulation(clientId, remoteApi.simx_opmode_oneshot);
        vrep.simxCloseScene(clientId, remoteApi.simx_opmode_oneshot_wait);
        String scene = System.getenv("GAMS_ROOT"); // need to have the $GAMS_ROOT environment variable set up
        scene += "/resources/vrep/starting_NoBoats.ttt";
        vrep.simxLoadScene(clientId, scene, 0, remoteApi.simx_opmode_oneshot_wait);

        //Load functional water model
        String WATER_FILE = System.getenv("GAMS_ROOT");
        WATER_FILE += "/resources/vrep/water_functional.ttm";
        IntW waterHandleW = new IntW(1);
        int returnVal = vrep.simxLoadModel(clientId, WATER_FILE, 0, 
                        waterHandleW, remoteApi.simx_opmode_oneshot_wait);
        if (returnVal != vrep.simx_return_ok) {
            System.out.println("Water functional failed to load");
            System.exit(-1);
        }

        //Add models to scene. Pass sw_corner as parameter
        ArrayList<vrepSimBoat> boats = new ArrayList<>();
        for (int i = 0; i < NUMBER_OF_SIMULATED_BOATS; i++) {
            System.out.println("Adding boat " + i);
            boats.add(new vrepSimBoat(i, initPositions[i], swPosition));
        }
        
        vrep.simxStartSimulation(clientId, returnVal);    ////////////////////////////////////////////////////////////////////////////////////
        
        boolean flag = true;
        //double tStart = System.currentTimeMillis()/1000.0;
        while (flag) {
            //double t = System.currentTimeMillis()/1000.0;
            //if ((t - tStart) > 5.0) {flag = false;}
        }
        // close connection to vrep
        vrep.simxStopSimulation(clientId, returnVal);
        System.out.print("closing vrep connection...");
        vrep.simxFinish(clientId);        
        System.out.println("done");

    }

}
