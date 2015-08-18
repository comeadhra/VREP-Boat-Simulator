package vrepSim;

import com.gams.utility.Position;

import org.apache.commons.math.linear.RealMatrix;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.EnumSet;
import java.util.Set;

enum SENSOR_CATEGORY {
    LOCALIZATION, ENVIRONMENTAL
}
enum SENSOR_TYPE {

    GPS(SENSOR_CATEGORY.LOCALIZATION),
    COMPASS(SENSOR_CATEGORY.LOCALIZATION),
    GYRO(SENSOR_CATEGORY.LOCALIZATION),
    IMU(SENSOR_CATEGORY.LOCALIZATION),
    DGPS(SENSOR_CATEGORY.LOCALIZATION),
    MOTOR(SENSOR_CATEGORY.LOCALIZATION),
    ES2(SENSOR_CATEGORY.ENVIRONMENTAL),
    EC(SENSOR_CATEGORY.ENVIRONMENTAL),
    TEMP(SENSOR_CATEGORY.ENVIRONMENTAL),
    DO(SENSOR_CATEGORY.ENVIRONMENTAL),
    WIFI(SENSOR_CATEGORY.ENVIRONMENTAL);

    SENSOR_CATEGORY category;

    SENSOR_TYPE(SENSOR_CATEGORY category) {
        this.category = category;
    }
    public static Set<SENSOR_TYPE> localization = EnumSet.of(GPS, COMPASS, GYRO, IMU, DGPS, MOTOR);
    public static Set<SENSOR_TYPE> environmental = EnumSet.of(ES2, EC, TEMP, DO, WIFI);

    public static int getEnvironmentalOrdinal(SENSOR_TYPE type) {
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
}

/**
 * @author jjb
 */
public class Datum {

    SENSOR_TYPE type;
    //Position position;
    Long timestamp;
    RealMatrix z; // sensor value
    RealMatrix R; // sensor covariance
    long id;
    int boatID;
    static long idIncrement = 0;
    static DateFormat df = new SimpleDateFormat("yy/MM/dd HH:mm:ss");
    Date dateobj;

    static LutraMadaraContainers containers;

    public Datum(SENSOR_TYPE type, Long timestamp, RealMatrix z, int boatID) {
        this.type = type;
        this.timestamp = timestamp;
        this.boatID = boatID;
        this.z = z.copy();
        this.id = idIncrement;
        idIncrement++;
        dateobj = new Date();
        
        //if (containers != null) {
        //    position = new Position();
        //    position.setX(containers.self.device.location.get(0));
        //    position.setY(containers.self.device.location.get(1));
        //}
        
    }
    public Datum(SENSOR_TYPE type, Long timestamp, RealMatrix z, RealMatrix R, int boatID) { // for localization data
        this(type,timestamp,z, boatID);
        this.R = R.copy();
    }
    public Datum(SENSOR_TYPE type, Position position, Long timestamp, RealMatrix z, int boatID) { // for environmental data
        this(type,timestamp,z, boatID);
        //this.position = position;
    }

    static void setContainersObject(LutraMadaraContainers inputContainers) {
        containers = inputContainers;
    }

    public void setZ(RealMatrix z) {this.z = z.copy(); }
    public void setR(RealMatrix R) {this.R = R.copy(); }
    //public void setPosition(Position position) {this.position = position;}
    public void setTimestamp(Long timestamp) {this.timestamp = timestamp;}
    public void setType(SENSOR_TYPE type) {this.type = type;}
    public RealMatrix getZ() {return this.z.copy();}
    public SENSOR_TYPE getType() {return this.type;}
    public RealMatrix getR() {return this.R.copy();}
    public Long getTimestamp() {return this.timestamp;}
    //public Position getPosition() {return this.position;}
    
    @Override
    public String toString() {
        return String.format("TYPE = %s,  DATE = 20%s,  LAT = %.6e,  LONG = %.6e, VALUE = %s",
                typeString(this.type),df.format(dateobj),containers.self.device.location.get(0),containers.self.device.location.get(1),zString());
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
        if (type == SENSOR_TYPE.ES2) {
            return "ES2";
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
        // put everything into the environmentalData FlexMap in LutraMadaraContainers

        //long currentCount = ;


    }


}
