package vrepSim;

import com.gams.algorithms.BaseAlgorithm;
import com.gams.controllers.BaseController;


/**
 * Simple algorithm: continually try to maintain current global location
 *
 * @author jjb
 *
 */
public class DwellAlgorithm extends BaseAlgorithm {

    public DwellAlgorithm() {
    }

    @Override
    public void init(BaseController controller) {
        super.init(controller);

    }

    @Override
    public int analyze() {
        return 0;
    }

    @Override
    public int plan() {


        return 0;
    }

    @Override
    public int execute() {

        return 0;
    }


    public void shutdown() {
        // Free MADARA containers
    }
}
