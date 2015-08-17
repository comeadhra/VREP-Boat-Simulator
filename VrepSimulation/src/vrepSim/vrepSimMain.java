package vrepSim;

import com.gams.utility.GpsPosition;
import java.util.ArrayList;
import coppelia.remoteApi;

public class vrepSimMain{
    public static void main(String[] args)
    {
      //Create remote API object
      remoteApi vrep = new remoteApi();
      
      //Number of boats in the scene
      final int NUMBER_OF_SIMULATED_BOATS = 1;
      //Initial positions of boats
      GpsPosition[] initPositions = new GpsPosition[NUMBER_OF_SIMULATED_BOATS];
      initPositions[0] =  new GpsPosition(0,0,0);
      
      //Vrep communication settings
      String vrepHost = "127.0.0.1";
      int vrepPort = 19905;
      
      //GPS co-ordinates of VREP origin
      double sw_lat = 40.442824;
      double sw_long = -79.940967;

      GpsPosition swPosition = new GpsPosition(sw_lat, sw_long, 0.0);
      
      //Open vrep connection
      System.out.print("connecting to vrep...");
      int clientId = 
        vrep.simxStart (vrepHost, vrepPort, true, true, 2000, 5);
      if (clientId == -1)
      {
        System.out.println("failure connecting to vrep");
        System.exit (-1);
      }

      //load simultion scene
      vrep.simxStopSimulation (clientId, remoteApi.simx_opmode_oneshot);
      vrep.simxCloseScene (clientId, remoteApi.simx_opmode_oneshot_wait);
      String scene  = System.getenv ("GAMS_ROOT"); // need to have the $GAMS_ROOT environment variable set up
      scene += "/resources/vrep/starting.ttt";
      vrep.simxLoadScene (clientId, scene, 0, remoteApi.simx_opmode_oneshot_wait);

      //Add models to scene. Pass sw_corner as parameter
      ArrayList<vrepSimBoat> boats = new ArrayList<>();
      for(int i = 0; i < NUMBER_OF_SIMULATED_BOATS; i++)
          boats.add( new vrepSimBoat(i, initPositions[i], swPosition) );
      boolean flag = true;
      while(flag){/**/};  
      // close connection to vrep
      System.out.print("closing vrep connection...");
      vrep.simxFinish (clientId);
      System.out.println("done");

    }

}
