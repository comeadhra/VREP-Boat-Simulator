package vrepSim;

/**
 * @author jjb
 */
public class VelocityMotorMap {

    LutraMadaraContainers containers;
    final double MIN_DESIRED_VELOCITY = 1.52;
    final double MAX_DESIRED_VELOCITY = 2.45;
    final double MIN_THRUST_FRACTION = 0.1;
    final double MAX_THRUST_FRACTION = 1.0;


    VelocityMotorMap(LutraMadaraContainers containers) {
        this.containers = containers;
    }

    // the inverse map --> invert the model fit to the collected data
    double velocityToThrustFraction(double desiredVelocity) {
        // enforce bounds on desiredVelocity
        if (desiredVelocity < MIN_DESIRED_VELOCITY) {
            System.out.println("WARNING: input desired velocity is less than 1.52. Returning thrust fraction = 0 ...");
            return 0.0;
        }
        if (desiredVelocity > MAX_DESIRED_VELOCITY) {
            System.out.println("WARNING: input desired velocity is more than 2.45. Using velocity = 2.45 ...");
            desiredVelocity = MAX_DESIRED_VELOCITY;
        }
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            return Math.exp((desiredVelocity - 2.45)/0.4047);

        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
        }

        System.out.println("Unknown thrust type. Returning 0 ...");
        return 0.0;
    }

    // the forward map --> this is the model fit to the collected data
    double thrustFractionToVelocity(double thrustFraction) {
        // enforce bounds on thrustFraction
        if (thrustFraction < MIN_THRUST_FRACTION)  {
            if (thrustFraction > 0.0) {
                //System.out.println("WARNING: input thrust fraction is less than 0.1. Returning velocity = 0 ...");
            }
            return 0.0;
        }
        if (thrustFraction > 1.0) {
            System.out.println("WARNING: input thrust fraction is lmore than 1.0. Using thrust fraction of 1.0 ...");
            thrustFraction = 1.0;
        }
        if (containers.thrustType.get() == THRUST_TYPES.DIFFERENTIAL.getLongValue()) {
            return 0.4047*Math.log(thrustFraction) + 2.45;
        }
        else if (containers.thrustType.get() == THRUST_TYPES.VECTORED.getLongValue()) {
        }

        System.out.println("Unknown thrust type. Returning 0 ...");
        return 0.0;
    }



}
