package vrepSim;

import com.madara.threads.BaseThread;
import com.madara.threads.Threader;
import java.util.ArrayList;
import java.util.Random;
import org.apache.commons.math.linear.MatrixUtils;
import com.gams.utility.Position;
import org.apache.commons.math.linear.RealMatrix;

enum SPOOFED_SENSOR_SETTINGS {
    
    DO(50.0, 0.01, 10.0, 0.0, 1.0, 0.01, 1.0, true, 40.0),
    EC(300.0, 0.01, 10.0, 0.0, 1.0, 0.01, 1.0, false, 0.0),
    T(20.0, 0.01, 10.0, 0.0, 1.0, 0.01, 1.0, true, 10.0);
    
    double base;
    double distFactor;
    double valueFactor;
    double sigmaIncreaseRate;
    double valueDecreaseRate;
    double addRate;
    double noiseLevel;
    boolean hysteresis;
    double hysteresisRate;

    private SPOOFED_SENSOR_SETTINGS(double base, double distFactor, 
            double valueFactor, double sigmaIncreaseRate, 
            double valueDecreaseRate, double addRate, 
            double noiseLevel, boolean hysteresis, double hysteresisRate) {
        this.base = base;
        this.distFactor = distFactor;
        this.valueFactor = valueFactor;
        this.sigmaIncreaseRate = sigmaIncreaseRate;
        this.valueDecreaseRate = valueDecreaseRate;
        this.addRate = addRate;
        this.noiseLevel = noiseLevel;
        this.hysteresis = hysteresis;
        this.hysteresisRate = hysteresisRate;
        
        // TODO: implement something with hysteresis
    }        
}


/**
 *
 * @author jjb
 */
public class SpoofedSensor {

    public static final long START_TIME = System.currentTimeMillis();
    public static final Random RANDOM = new Random(START_TIME);    
    
    ArrayList<Double> xs;
    ArrayList<Double> ys;
    ArrayList<Double> vs;
    ArrayList<Double> sigmas;
    SPOOFED_SENSOR_SETTINGS settings;
    double currentLat;
    double currentLon;
    DatumListener listener;
    SENSOR_TYPE type;
    SensorGenerationThread thread;
    String threadName;
    int boatID;
    
    public SpoofedSensor(SENSOR_TYPE type, DatumListener listener, int boatID, double originalLat, double originalLon) {
        xs = new ArrayList<Double>();
        ys = new ArrayList<Double>();
        vs = new ArrayList<Double>();
        sigmas = new ArrayList<Double>();
        this.listener = listener;
        this.type = type;
        thread = new SensorGenerationThread();
        this.boatID = boatID;
        
        if (type == SENSOR_TYPE.DO) {
            settings = SPOOFED_SENSOR_SETTINGS.DO;
            threadName = "DO_Sensor";
        }
        else if (type == SENSOR_TYPE.EC) {
            settings = SPOOFED_SENSOR_SETTINGS.EC;
            threadName = "EC_Sensor";
        }
        else if (type == SENSOR_TYPE.TEMP) {
            settings = SPOOFED_SENSOR_SETTINGS.T;
            threadName = "T_Sensor";
        }
        currentLat = originalLat;
        currentLon = originalLon;
        xs.add(currentLon);
        ys.add(currentLat);
        vs.add(settings.base + settings.distFactor * (RANDOM.nextDouble() - 0.5) + settings.noiseLevel*(RANDOM.nextDouble() - 0.5));
        sigmas.add(0.01);
    }        
    
    class SensorGenerationThread extends BaseThread {
        @Override
        public void run() {            
            RealMatrix z = MatrixUtils.createRealMatrix(1, 1);
            z.setEntry(0,0,generateValue(currentLat,currentLon));
            Datum datum = new Datum(type,System.currentTimeMillis(),z,currentLat,currentLon,boatID);            
            listener.newDatum(datum);
        }        
    }   
        
    
    public double generateValue(double lat, double lon) {
        currentLat = lat;
        currentLon = lon;
        double result = 0.0;
        
        // possibly generate new gaussian peaks, up to 20
        if ((RANDOM.nextDouble() < settings.addRate && xs.size() < 20)) {
              double newlon = currentLon + settings.distFactor * (RANDOM.nextDouble() - 0.5);
              double newlat = currentLat + settings.distFactor * (RANDOM.nextDouble() - 0.5);
              double value = RANDOM.nextDouble() * settings.valueFactor;
              if (RANDOM.nextBoolean()) {
                  value = -value;
              }

              xs.add(newlon);
              ys.add(newlat);
              vs.add(value);
              sigmas.add(0.01);
        }
        
        result = computeGTValue(lat,lon);

        return result;
    }    
    
    public double computeGTValue(double lat, double lon) {
        double v = settings.base;
        synchronized (xs) {
            for (int i = 0; i < xs.size(); i++) {

                double dx = xs.get(i) - lon;
                double dy = ys.get(i) - lat;
                double distSq = dx * dx + dy * dy;
                double noise = settings.noiseLevel*RANDOM.nextGaussian();

                double dv = vs.get(i) * (1.0 / Math.sqrt(2.0 * Math.PI * sigmas.get(i) * sigmas.get(i))) * Math.pow(Math.E, -(distSq / (2.0 * sigmas.get(i) * sigmas.get(i))));
                v += dv + noise;
            }
        }
        return v;
    }    
}
