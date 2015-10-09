package vrepSim;

import com.gams.utility.Position;

import org.apache.commons.math.linear.RealMatrix;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.EnumSet;
import java.util.Set;

enum SENSOR_CATEGORY {
    LOCALIZATION, ENVIRONMENTAL
}
enum SENSOR_TYPE {

    // TODO: verify the sampling rate for all the sensor types
    GPS(SENSOR_CATEGORY.LOCALIZATION,false,5.0),
    COMPASS(SENSOR_CATEGORY.LOCALIZATION,false,10.0),
    GYRO(SENSOR_CATEGORY.LOCALIZATION,false,10.0),
    IMU(SENSOR_CATEGORY.LOCALIZATION,false,10.0),
    DGPS(SENSOR_CATEGORY.LOCALIZATION,false,5.0),
    MOTOR(SENSOR_CATEGORY.LOCALIZATION,false,25.0),
    EC(SENSOR_CATEGORY.ENVIRONMENTAL,false,20.0),
    TEMP(SENSOR_CATEGORY.ENVIRONMENTAL,true,20.0),
    DO(SENSOR_CATEGORY.ENVIRONMENTAL,true,5.0),
    WIFI(SENSOR_CATEGORY.ENVIRONMENTAL,false,1.0),
    DEPTH(SENSOR_CATEGORY.ENVIRONMENTAL,false,1.0),
    FLOW(SENSOR_CATEGORY.ENVIRONMENTAL,true,5.0);

    SENSOR_CATEGORY category;
    boolean hysteresis;
    double Hz;

    SENSOR_TYPE(SENSOR_CATEGORY category, boolean hysteresis, double Hz) {
        this.category = category;
        this.hysteresis = hysteresis;
        this.Hz = Hz;
    }
    public static Set<SENSOR_TYPE> localization = EnumSet.of(GPS, COMPASS, GYRO, IMU, DGPS, MOTOR);
    public static Set<SENSOR_TYPE> environmental = EnumSet.of(EC, TEMP, DO, WIFI, DEPTH, FLOW);

    /*
    public static int getEnvironmentalOrdinal(SENSOR_TYPE type) { // return index of environmental sensor --> not necessary if you use a HashMap
        SENSOR_TYPE[] types = SENSOR_TYPE.values();
        int environmentalSensorTypeCount = 0;
        for (int i = 0; i < types.length; i++) {
            if (types[i].category == SENSOR_CATEGORY.ENVIRONMENTAL) {
                if (type == types[i]) {
                    return environmentalSensorTypeCount;
                }
                environmentalSensorTypeCount++;
            }
        }
        return -1;
    }
    */
}

/**
 * @author jjb
 */
public class Datum {

    SENSOR_TYPE type;
    Position position;
    Long timestamp;
    RealMatrix z; // sensor value
    RealMatrix R; // sensor covariance
    long id;
    int boatID;
    static long idIncrement = 0;
    static DateFormat df = new SimpleDateFormat("yy/MM/dd HH:mm:ss");
    Date dateobj;

    static LutraMadaraContainers containers;
    private static FileOutputStream logFileWriter;
    //private static final Logger newLogger = LoggerFactory.getLogger();
    //private static final FileAppender fileAppender = new FileAppender();

    public Datum(SENSOR_TYPE type, Long timestamp, RealMatrix z, int boatID) {
        this.type = type;
        this.timestamp = timestamp;
        this.boatID = boatID;
        this.z = z.copy();
        this.id = idIncrement;
        idIncrement++;
        dateobj = new Date();
    }
    public Datum(SENSOR_TYPE type, Long timestamp, RealMatrix z, RealMatrix R, int boatID) { // for localization data
        this(type,timestamp,z, boatID);
        this.R = R.copy();
    }
    public Datum(SENSOR_TYPE type, Position position, Long timestamp, RealMatrix z, int boatID) { // for environmental data
        this(type,timestamp,z, boatID);
        this.position = position;
    }

    static void setContainersObject(LutraMadaraContainers inputContainers) {
        containers = inputContainers;
    }

    public void setZ(RealMatrix z) {this.z = z.copy(); }
    public void setR(RealMatrix R) {this.R = R.copy(); }
    public void setPosition(Position position) {this.position = position;}
    public void setTimestamp(Long timestamp) {this.timestamp = timestamp;}
    public void setType(SENSOR_TYPE type) {this.type = type;}
    public RealMatrix getZ() {return this.z.copy();}
    public SENSOR_TYPE getType() {return this.type;}
    public RealMatrix getR() {return this.R.copy();}
    public Long getTimestamp() {return this.timestamp;}
    public Position getPosition() {return this.position;}

    @Override
    public String toString() {
        return String.format("TYPE = %s,  DATE = %s,  TIME = %d,  LAT = %.6e,  LONG = %.6e, VALUE = %s",
                typeString(this.type),df.format(dateobj),timestamp,position.getX(),position.getY(),zString());
    }

    String zString () {
        String a = "[";
        for (int i = 0; i < z.getRowDimension()-1; i++) {
            a = a + String.format("%f,",z.getEntry(i,0));
        }
        a = a + String.format("%f]",z.getEntry(z.getRowDimension()-1,0));
        return a;
    }

    public boolean isType(SENSOR_TYPE type) {return this.type == type;}

    static public String typeString(SENSOR_TYPE type) {
        if (type == SENSOR_TYPE.GPS) {
            return "GPS";
        }
        if (type == SENSOR_TYPE.COMPASS) {
            return "COMPASS";
        }
        if (type == SENSOR_TYPE.GYRO) {
            return "GYRO";
        }
        if (type == SENSOR_TYPE.IMU) {
            return "IMU";
        }
        if (type ==  SENSOR_TYPE.DGPS) {
            return "DGPS";
        }
        if (type == SENSOR_TYPE.MOTOR) {
            return "MOTOR";
        }
        if (type == SENSOR_TYPE.EC) {
            return "EC";
        }
        if (type == SENSOR_TYPE.TEMP) {
            return "TEMP";
        }
        if (type == SENSOR_TYPE.DO) {
            return "DO";
        }
        if (type == SENSOR_TYPE.WIFI) {
            return "WIFI";
        }
        return "UNKNOWN";
    }

    public void toKnowledgeBase() {
        // TODO: put everything into the environmentalData FlexMap in LutraMadaraContainers

        //long currentCount = ;


    }

    private static String EnvironmentalLogFilename() {
        Date d = new Date();
        SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMdd_hhmmss");
        return "/mnt/sdcard/ENVIRONMENTAL_DATA_" + sdf.format(d) + ".txt";
    }

    public static void establishLogger() {
        //File logfile = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM);
        String filename = EnvironmentalLogFilename();
        //String path = logfile.getPath() + "/" + filename;
        try {
            logFileWriter = new FileOutputStream(filename);
        } catch (FileNotFoundException e) {
            //Log.e("jjb_DATA_LOGGER", e.toString() + ": failed to create " + filename);
        }

    }

    public void pushToLog() {

        String stringForLog = (toString() + "\n");

        try {
            //newLogger.info(toString());
            logFileWriter.write(stringForLog.getBytes());
        }catch(IOException e) {
        }catch (NullPointerException e){
        }
    }

    public static void endLog() {
        try {
            //newLogger.close();
            logFileWriter.close();
        }catch(IOException e) {
        }
    }


}
