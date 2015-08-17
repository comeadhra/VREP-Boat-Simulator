package vrepSim;

import java.util.EventListener;

/**
 * @author jjb
 */
public interface DatumListener extends EventListener {
    void newDatum(Datum datum); // update a new localization measurement
}
