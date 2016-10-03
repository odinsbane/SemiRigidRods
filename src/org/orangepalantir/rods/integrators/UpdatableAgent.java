package org.orangepalantir.rods.integrators;

/**
 * Created by matt on 03/10/16.
 */
public interface UpdatableAgent{

    int getForceCount();
    int getValueCount();
    void storePositions(double[] data, int offset);
    void storeForces(double[] data, int offset);
    void restorePositions(double[] data, int offset);
    void clearForces();
    void step(double dt);
    void restoreForces(double[] data, int offset);
    double prepareInternalForces();

}
