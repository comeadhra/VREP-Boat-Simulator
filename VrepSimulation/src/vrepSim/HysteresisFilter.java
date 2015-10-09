package vrepSim;

import com.madara.KnowledgeBase;
import com.madara.KnowledgeRecord;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * @author jjb
 */
public class HysteresisFilter implements DatumListener {

    HashMap<SENSOR_TYPE,List<Double>> heightsHashMap; // 5 numbers, using P squared algorithm for calculating median without storing values
    HashMap<SENSOR_TYPE,List<Integer>> markersHashMap;
    HashMap<SENSOR_TYPE,List<Double>> desiredMarkersHashMap;
    HashMap<SENSOR_TYPE,List<Double>> medianChangesHashMap;

    HashMap<SENSOR_TYPE,Boolean> convergedHashMap;
    HashMap<SENSOR_TYPE,Long> dataToKBCount; // number of data points pushed into knowledge base

    KnowledgeBase knowledge;
    LutraMadaraContainers containers;
    boolean dwelling;
    double oldMedian;

    final double percentile = 0.5;
    final double[] increment = new double[] {0, percentile/2.0, percentile, (1.0 + percentile)/2.0, 1};
    final double[] defaultDesiredMarkers = new double[] {1, 1+2*percentile, 1+4*percentile, 3+2*percentile, 5};
    final double WORST_ONE_SECOND_MEDIAN_CHANGE_ALLOWED = 0.25;

    HysteresisFilter(KnowledgeBase knowledge, LutraMadaraContainers containers) {
        this.knowledge = knowledge;
        this.containers = containers;
        dwelling = false;
        heightsHashMap = new HashMap<>();
        markersHashMap = new HashMap<>();
        desiredMarkersHashMap = new HashMap<>();
        convergedHashMap = new HashMap<>();
        dataToKBCount = new HashMap<>();
        for (SENSOR_TYPE type : SENSOR_TYPE.environmental) {
            convergedHashMap.put(type,!type.hysteresis); // non-hysteresis sensors are inherently converged
        }
        oldMedian = 1e30; // humongous value rather than infinity so weird NaN stuff doesn't happen
    }

    @Override
    public void newDatum(Datum datum) {
        SENSOR_TYPE type = datum.getType();
        String logString = datum.toString();

        //Log.i("jjb_HYSTERESIS",String.format("New datum: %s",datum.toString()));

        if (!isConverged(type)) {
            logString = logString + " -- WARNING: MAY HAVE HYSTERESIS";
            filter(datum);
        }
        else {
            datum.toKnowledgeBase(); //push into knowledge base
            incrementCount(datum.getType());
        }
        datum.pushToLog();
    }

    public void filter(Datum datum) {
        if (containers.distToDest.get() < containers.sufficientProximity.get()) {
            if (!dwelling) {
                dwelling = true;
                resetAll();
            }
            checkForConvergence(datum);
        }
        else {
            dwelling = false;
            convergedHashMap.put(datum.getType(), false);
        }
    }

    void checkForConvergence(Datum datum) {

        //Log.i("jjb_HYSTERESIS","new iteration of checkForConvergence() ");

        SENSOR_TYPE type = datum.getType();
        List<Double> heights = heightsHashMap.get(type);
        List<Integer> markers = markersHashMap.get(type);
        List<Double> desiredMarkers = desiredMarkersHashMap.get(type);
        List<Double> medianChanges = medianChangesHashMap.get(type);
        double newValue = datum.getZ().getEntry(0,0);
        if (heights.size() < 5) {
            heights.add(newValue);
            return;
        }
        if (heights.size() == 5) { // need to sort initial list of five values in ASCENDING order
            Collections.sort(heights);
            return;
        }

        boolean converged = false;

        double signSum = 0;
        int k = 0;
        for (int i = 0; i < 5; i++) {
            signSum += Math.signum(newValue - heights.get(i));
        }
        if (signSum == -5) {
            k = 1;
            heights.set(1,newValue);
        }
        else if ((signSum == -4) || (signSum == -3)) {
            k = 1;
        }
        else if ((signSum == -2) || (signSum == -1)) {
            k = 2;
        }
        else if ((signSum == 0) || (signSum == 1)) {
            k = 3;
        }
        else if ((signSum == 2) || (signSum == 3)) {
            k = 4;
        }
        else {
            k = 4;
            heights.set(5,newValue);
        }

        for (int i = k; i < 5; i++) {
            markers.set(i,markers.get(i) + 1);
        }
        for (int i = 1; i < 5; i++) {
            desiredMarkers.set(i,desiredMarkers.get(i) + increment[i]);
        }

        for (int j = 1; j < 4; j++) {
            double d = desiredMarkers.get(j) - markers.get(j);
            if ((d >= 1 && (markers.get(j+1)-markers.get(j)) > 1) || (d <= -1 && (markers.get(j-1) - markers.get(j)) < -1)) {
                d = Math.signum(d);
                double candidateHeight = heights.get(j) + d/(markers.get(j+1)-markers.get(j-1))*
                    ((markers.get(j)-markers.get(j-1)+d)*(heights.get(j+1)-heights.get(j))/(markers.get(j+1)-markers.get(j)) +
                    (markers.get(j+1)-markers.get(j)-d)*(heights.get(j)-heights.get(j-1))/(markers.get(j)-markers.get(j-1)));
                if ((heights.get(j-1) < candidateHeight) && (candidateHeight < heights.get(j+1))) {
                    heights.set(j,candidateHeight);
                }
                else {
                    heights.set(j,heights.get(j) + d*(heights.get(j + (int)d) - heights.get(j))/(markers.get(j + (int)d) - markers.get(j)));
                }
                markers.set(j,markers.get(j) + (int)d);
            }
        }

        double median = heights.get(3);
        double medianChange = median - oldMedian;
        oldMedian = median;

        if (Math.abs(medianChange) > 0) {
            medianChanges.add(medianChange);
            if (medianChanges.size() > type.Hz) {
                medianChanges.remove(1); // only maintain last second's worth of measurements
            }

            double max = -1e30;
            for (Double change : medianChanges) {
                max = Math.max(max,Math.abs(change));
            }
            if (max < WORST_ONE_SECOND_MEDIAN_CHANGE_ALLOWED) {
                converged = true;
            }
        }

        if (converged) {
            convergedHashMap.put(datum.getType(), true);
        } else {
            convergedHashMap.put(datum.getType(), false);
        }
    }


    void resetAll() {
        for (Map.Entry<SENSOR_TYPE, List<Double>> entry : heightsHashMap.entrySet()) {
            resetSpecific(entry.getKey());
        }
    }
    void resetSpecific(SENSOR_TYPE type) {
        heightsHashMap.get(type).clear();
        markersHashMap.get(type).clear();
        desiredMarkersHashMap.get(type).clear();
        for (int i = 0; i < 5; i++) {
            markersHashMap.get(type).add(i);
            desiredMarkersHashMap.get(type).add(defaultDesiredMarkers[i]);
        }
    }
    void allUnconverged() {
        for (Map.Entry<SENSOR_TYPE, Boolean> entry : convergedHashMap.entrySet()) {
            entry.setValue(false);
        }
    }
    boolean isConverged(SENSOR_TYPE type) {
        return convergedHashMap.get(type);
    }
    void incrementCount(SENSOR_TYPE type) {
        if (!dataToKBCount.containsKey(type)) {
            dataToKBCount.put(type,1L);
        }
        else {
            dataToKBCount.put(type, dataToKBCount.get(type) + 1);
        }
    }


}
