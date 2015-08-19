package vrepSim;

import com.gams.utility.Position;
import com.gams.variables.Self;

import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;
import com.madara.UpdateSettings;
import com.madara.containers.Counter;
import com.madara.containers.Double;
import com.madara.containers.FlexMap;
import com.madara.containers.Integer;
import com.madara.containers.DoubleVector;
import com.madara.containers.IntegerVector;
import com.madara.containers.NativeDoubleVector;
import com.madara.containers.String;

import org.apache.commons.math.linear.MatrixUtils;
import org.apache.commons.math.linear.RealMatrix;

import org.jscience.geography.coordinates.LatLong;
import org.jscience.geography.coordinates.UTM;
import org.jscience.geography.coordinates.crs.ReferenceEllipsoid;

import java.util.HashMap;

import javax.measure.unit.NonSI;
import javax.measure.unit.SI;

enum THRUST_TYPES {
    VECTORED(0), DIFFERENTIAL(1), TWO_FAN_CATAMARAN(2);
    private final long value;
    THRUST_TYPES(long value) { // constructor allows for special types in the enum
        this.value = value;
    }
    public final long getLongValue() { // need a get function to access the values
        return value;
    }
}

enum TELEOPERATION_TYPES {
    NONE(0), GUI_WP(1), GUI_MS(2), RC(3), BEACON(4);
    // NONE --> control loops are always active (always try to arrive at and stay at current destination unless algorithm overrides)
    // GUI_WP --> user selects waypoint(s) in a GUI. Boat controls arrival, but the boat does nothing after it arrives
    // GUI_MS --> user is sending motor signals to the boats via a GUI (w/ joystick). Boat control loops completely disabled
    // RC --> user is sending motor signals to the boats via a radio controller. Boat control loops completely disabled
    // BEACON --> user controls location of a virtual beacon that boats gather around while holding formation
    private final long value;
    TELEOPERATION_TYPES(long value) { this.value = value; }
    public final long getLongValue() { return value; }
    //TODO: implement the various teleoperation capabilities
}

/**
 * @author jjb
 */
public class LutraMadaraContainers {

    // Pass in knowledge base
    // Takes care of all the SetName's for all the code at once - no more hard coding strings in multiple locations!
    // Then just pass this object around in the constructors for things like the EFK and controller

    KnowledgeBase knowledge;
    java.lang.String prefix;
    UpdateSettings settings; // used to force a global variable to not broadcast as if it were local
    // allows other agents to change this value but does not broadcast changes made locally
    // e.g. i want to teleoperate the boat by changing the motor commands directly from the GUI agent

    String unhandledException;
    Double distToDest;
    Double sufficientProximity;
    Double peakVelocity;
    Double accel;
    Double decel;
    DoubleVector localState;
    NativeDoubleVector eastingNorthingBearing; // UTM x,y,th
    NativeDoubleVector errorEllipse; // width, height, angle
    NativeDoubleVector localStateXYCovariance; // P(0,0), P(1,0), P(0,1), P(1,1)
    Integer longitudeZone;
    String latitudeZone; // a single character (see UTM) http://jscience.org/api/org/jscience/geography/coordinates/UTM.html
    Integer executingProfile; // == 1 if controller is currently executing a velocity profile, == 0 otherwise
    Integer thrustType; // see THRUST_TYPES enum
    Integer teleopStatus; // see TELEOPERATION_TYPES enum
    Integer gpsInitialized; // == 1 if the first GPS lock has come in
    Integer compassInitialized; // == 1 if the first compass measurement has come in
    Integer localized; // == 1 if both GPS and compass are initialized
    Integer resetLocalization; // operator will temporarily set this to 1 to force the boat to totally reset its local state
    Integer connectivityWatchdog;
    Integer wifiStrength;
    NativeDoubleVector motorCommands;
    Double thrustFraction;
    Double bearingFraction;
    NativeDoubleVector bearingPIDGains;
    NativeDoubleVector thrustPIDGains;
    NativeDoubleVector thrustPPIGains;
    final double defaultSufficientProximity = 3.0;
    final double defaultPeakVelocity = 2.0;
    final double defaultAccelTime = 5.0;
    final double defaultDecelTime = 5.0;
    final double maxAccel = 1.0; // no more than X m/s^2 capable at full power
    final double minAccel = 0.1; // no less than X m/s^2, or motor doesn't respond
    final long defaultTeleopStatus = TELEOPERATION_TYPES.GUI_MS.getLongValue(); // start in teleop mode!
    final long defaultThrustType = THRUST_TYPES.DIFFERENTIAL.getLongValue();
    final double controlHz = 25.0; // frequency of control loop and sending the corresponding JSON commands
    final double[] bearingPIDGainsDefaults = new double[]{0.25,0.0,200.0}; // cols: P,I,D
    final double[] thrustPIDGainsDefaults = new double[]{0.2,0,0.3}; // cols: P,I,D
    final double[] thrustPPIGainsDefaults = new double[]{0.2,0.2,0.2}; // cols: Pos-P, Vel-P, Vel-I

    //static long[] environmentalDataCount = new long[SENSOR_TYPE.environmental.size()];
    HashMap<SENSOR_TYPE, Long> environmentalDataCount = new HashMap<>();

    Self self;

    public LutraMadaraContainers(KnowledgeBase knowledge, Self self, THRUST_TYPES thrustType) {
        this.knowledge = knowledge;
        this.self = self;
        this.prefix = java.lang.String.format("device.%d.",this.self.id.get());
        this.settings = new UpdateSettings();
        settings.setTreatGlobalsAsLocals(true);

        this.self.device.dest.resize(3);
        this.self.device.home.resize(3);
        this.self.device.location.resize(3);
        this.self.device.source.resize(3);
        distToDest = new Double();
        distToDest.setName(knowledge, prefix + "distToDest");
        distToDest.setSettings(settings);
        sufficientProximity = new Double();
        sufficientProximity.setName(knowledge, prefix + "sufficientProximity");
        sufficientProximity.setSettings(settings);
        peakVelocity = new Double();
        peakVelocity.setName(knowledge, prefix + "peakVelocity");
        peakVelocity.setSettings(settings);
        accel = new Double();
        accel.setName(knowledge, prefix + "accelTime");
        accel.setSettings(settings);
        decel = new Double();
        decel.setName(knowledge, prefix + "decelTime");
        decel.setSettings(settings);
        localState = new DoubleVector();
        localState.setName(knowledge, prefix + "localState");
        localState.setSettings(settings);
        executingProfile = new Integer();
        executingProfile.setName(knowledge, prefix + "executingProfile");
        executingProfile.set(0);
        this.thrustType = new Integer();
        this.thrustType.setName(knowledge, prefix + "thrustType");
        this.thrustType.set(thrustType.getLongValue());
        motorCommands = new NativeDoubleVector();
        motorCommands.setName(knowledge, prefix + "motorCommands");
        motorCommands.setSettings(settings);
        motorCommands.resize(2);
        teleopStatus = new Integer();
        teleopStatus.setName(knowledge, prefix + "teleopStatus");
        //teleopStatus.setSettings(settings);
        gpsInitialized = new Integer();
        gpsInitialized.setName(knowledge, prefix + "gpsInitialized");
        gpsInitialized.set(0);
        compassInitialized = new Integer();
        compassInitialized.setName(knowledge, prefix + "compassInitialized");
        compassInitialized.set(0);
        localized = new Integer();
        localized.setName(knowledge, prefix + "localized");
        localized.set(0);
        longitudeZone = new Integer();
        longitudeZone.setName(knowledge, prefix + "longitudeZone");
        longitudeZone.setSettings(settings);
        latitudeZone = new String();
        latitudeZone.setName(knowledge, prefix + "latitudeZone");
        latitudeZone.setSettings(settings);
        eastingNorthingBearing = new NativeDoubleVector();
        eastingNorthingBearing.setName(knowledge, prefix + "eastingNorthingBearing");
        eastingNorthingBearing.resize(3);

        errorEllipse = new NativeDoubleVector();
        errorEllipse.setName(knowledge, prefix + "errorEllipse");
        errorEllipse.resize(3);

        localStateXYCovariance = new NativeDoubleVector();
        localStateXYCovariance.setName(knowledge, prefix + "localStateXYCovariance");
        localStateXYCovariance.setSettings(settings);
        localStateXYCovariance.resize(4);

        bearingPIDGains= new NativeDoubleVector();
        bearingPIDGains.setName(knowledge, prefix + "bearingPIDGains");
        bearingPIDGains.setSettings(settings);
        bearingPIDGains.resize(3);
        thrustPIDGains= new NativeDoubleVector();
        thrustPIDGains.setName(knowledge, prefix + "thrustPIDGains");
        thrustPIDGains.setSettings(settings);
        thrustPIDGains.resize(3);
        thrustPPIGains= new NativeDoubleVector();
        thrustPPIGains.setName(knowledge, prefix + "thrustPPIGains");
        thrustPPIGains.setSettings(settings);
        thrustPPIGains.resize(3);

        thrustFraction = new Double();
        bearingFraction = new Double();
        thrustFraction.setName(knowledge, prefix + "thrustFraction");
        bearingFraction.setName(knowledge, prefix + "bearingFraction");

        unhandledException = new String();
        unhandledException.setName(knowledge, prefix + "unhandledException");

        resetLocalization = new Integer();
        resetLocalization.setName(knowledge, prefix + "resetLocalization");

        connectivityWatchdog = new Integer();
        connectivityWatchdog.setName(knowledge, prefix + "connectivityWatchdog");
        connectivityWatchdog.set(1L); // boat sets to 1, GUI sets to 0, if the GUI doesn't see a 1, there is an issue with the connection

        wifiStrength = new Integer();
        wifiStrength.setName(knowledge, prefix + "wifiStrength");

        restoreDefaults();

        settings.free(); // don't need this object past the initialization
    }

    public void freeAll() {
        unhandledException.free();
        distToDest.free();
        sufficientProximity.free();
        peakVelocity.free();
        accel.free();
        decel.free();
        localState.free();
        executingProfile.free();
        teleopStatus.free();
        gpsInitialized.free();
        compassInitialized.free();
        localized.free();
        connectivityWatchdog.free();
        wifiStrength.free();
        motorCommands.free();
        longitudeZone.free();
        latitudeZone.free();
        bearingPIDGains.free();
        thrustPIDGains.free();
        thrustPPIGains.free();
        thrustFraction.free();
        bearingFraction.free();
        resetLocalization.free();
        errorEllipse.free();
        localStateXYCovariance.free();
    }

    public void restoreDefaults() {
        unhandledException.set("none");
        sufficientProximity.set(defaultSufficientProximity);
        peakVelocity.set(defaultPeakVelocity);
        accel.set(defaultAccelTime);
        decel.set(defaultDecelTime);
        teleopStatus.set(defaultTeleopStatus);
        //thrustType.set(defaultThrustType);
        resetLocalization.set(0);
        for (int i = 0; i < 3; i++) {
            bearingPIDGains.set(i,bearingPIDGainsDefaults[i]);
            thrustPIDGains.set(i,thrustPIDGainsDefaults[i]);
            thrustPPIGains.set(i,thrustPPIGainsDefaults[i]);
        }
    }

    public RealMatrix NDV_to_RM(NativeDoubleVector NDV) {
        // NativeDoubleVector to RealMatrix column conversion
        // Useful for pulling out self.device.home, .location, .dest, and .source
        double[] DA = NDV_to_DA(NDV);
        RealMatrix result = MatrixUtils.createColumnRealMatrix(DA);
        return result;
    }

    public double[] NDV_to_DA(NativeDoubleVector NDV) {
        // NativeDoubleVector to double[] conversion
        // Useful for pulling out self.device.home, .location, .dest, and .source
        // e.g. double[] home = containers.NDV_to_DA(self.device.home);
        KnowledgeRecord KR = NDV.toRecord();
        double[] result = KR.toDoubleArray();
        KR.free();
        return result;
    }

    public double velocityTowardGoal() {
        // calculate the boat's current velocity along the line between its current location and the goal
        RealMatrix initialV = MatrixUtils.createRealMatrix(2,1);
        initialV.setEntry(0,0,this.localState.get(3)*Math.cos(this.localState.get(2)) - this.localState.get(5));
        initialV.setEntry(1,0,this.localState.get(3)*Math.sin(this.localState.get(2)) - this.localState.get(6));
        RealMatrix xd = NDV_to_RM(self.device.dest).subtract(NDV_to_RM(self.device.home));
        RealMatrix x = MatrixUtils.createRealMatrix(2, 1);
        x.setEntry(0, 0, this.localState.get(0));
        x.setEntry(1,0,this.localState.get(1));
        RealMatrix xError = xd.getSubMatrix(0,1,0,0).subtract(x);
        RealMatrix xErrorNormalized = xError.scalarMultiply(1 / RMO.norm2(xError));
        double v = RMO.dot(initialV, xErrorNormalized); // initial speed in the direction of the goal
        return v;
    }

    public double[] UTMToLocalXY(UTM utm) {
        double[] result = new double[2];
        double[] home = NDV_to_DA(self.device.home);
        result[0] = utm.eastingValue(SI.METER) - home[0];
        result[1] = utm.northingValue(SI.METER) - home[1];
        return result;
    }

    public double[] PositionToLocalXY(Position position) {
        double [] result;
        // Convert from lat/long to NATO UTM coordinates
        UTM utmLoc = UTM.latLongToUtm(
                LatLong.valueOf(position.getX(), position.getY(), NonSI.DEGREE_ANGLE),
                ReferenceEllipsoid.WGS84);
        result = UTMToLocalXY(utmLoc);
        return result;
    }

    public LatLong LocalXYToLatLong() {
        UTM utm = LocalXYToUTM();
        LatLong latLong = UTM.utmToLatLong(utm, ReferenceEllipsoid.WGS84);
        return latLong;
    }

    public UTM LocalXYToUTM() {
        double easting = localState.get(0) + self.device.home.get(0);
        double northing = localState.get(1) + self.device.home.get(1);
        int longitudeZone = (int)this.longitudeZone.get();
        char latitudeZone = this.latitudeZone.get().charAt(0);
        UTM utm = UTM.valueOf(longitudeZone, latitudeZone, easting, northing, SI.METER);
        return utm;
    }





}
