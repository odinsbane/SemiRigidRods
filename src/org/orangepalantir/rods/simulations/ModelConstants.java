package org.orangepalantir.rods.simulations;

import java.nio.file.Path;

/**
 * Created on 29.03.17.
 */
public class ModelConstants {
    int ACTINS =    200;
    int MYOSINS = 25;
    double S0 = 0.1;
    double WIDTH = 25;
    double ACTIN_LENGTH = 2.0;
    double ACTIN_BENDING_STIFFNESS=0.0166;

    double ACTIN_ELASTIC_MODULOUS = 200;
    double THICKNESS = 0.2;
    double ANGLE_SIGMA = Math.PI/8;
    double MYOSIN_LENGTH = 0.8;
    double MYOSIN_BIND_LENGTH = 0.2;
    double MYOSIN_BIND_TIME = 75;

    double MYOSIN_STEPPING_FORCE = 0.5;
    double CROSSLINKER_LENGTH = 0.2;
    double SPRING_STIFFNESS = 100;

    double CROSSLINKER_BIND_FRACTION = 0.6;

    /**
     * For loading constants from a file.
     * @param p
     * @return
     */
    static ModelConstants loadConstants(Path p){
        ModelConstants m = new ModelConstants();
        return m;
    }
}
