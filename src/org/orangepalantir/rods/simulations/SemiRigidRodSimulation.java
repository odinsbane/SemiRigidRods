package org.orangepalantir.rods.simulations;

import org.orangepalantir.rods.Motor;
import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.RodViewer;
import org.orangepalantir.rods.Vector;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Created on 29.03.17.
 */
public class SemiRigidRodSimulation {
    List<RigidRod> actins;
    ModelConstants constants = new ModelConstants();
    Random ng;
    public void initializeSimulation(Random ng){
        this.ng = ng;
        actins = new ArrayList<>(constants.ACTINS);
        seedActinFilaments();
    }

    public void placeBoundMotor(Motor motor){

    }
    public void seedCrosslinkers(){
        List<RigidRod[]> crosslinkPairs = new ArrayList<>();
        for(int i = 0; i<actins.size(); i++){
            RigidRod a = actins.get(i);
            for(int j = i+1; j<actins.size(); j++){
                RigidRod b = actins.get(j);
                double separation = a.getClosestApproach(b, constants.WIDTH);
            }
        }
    }

    public RigidRod createNewActinFilament(Point center, Vector direction){
        int N = (int)(constants.ACTIN_LENGTH/constants.S0);
        RigidRod rod = new RigidRod(center, direction, N, constants.ACTIN_LENGTH);
        return rod;
    }


    public void seedActinFilaments(){
        for(int i = 0; i<constants.ACTINS; i++){

            //generate a free filament.
            double x = constants.WIDTH * ng.nextDouble() - 0.5 * constants.WIDTH;
            double y = constants.WIDTH * ng.nextDouble() - 0.5 * constants.WIDTH;
            double z = constants.THICKNESS * ng.nextDouble() - 0.5 * constants.THICKNESS;

            double theta = 2 * Math.PI * ng.nextDouble();
            double phi = Math.acos(Math.sin(constants.ANGLE_SIGMA/2)*(1 - 2*ng.nextDouble()));


            double nx = Math.cos(theta) * Math.sin(phi);
            double ny = Math.sin(theta) * Math.sin(phi);
            double nz = Math.cos(phi);

            RigidRod rod = createNewActinFilament(new Point(x, y, z), new Vector(nx, ny, nz));

            actins.add(rod);



        }
    }

    public static void main(String[] args){
        SemiRigidRodSimulation sim = new SemiRigidRodSimulation();
        sim.initializeSimulation(new Random(1));
        RodViewer viewer = new RodViewer();
        sim.actins.forEach(viewer::addRod);
        viewer.buildGui();
        viewer.repaint();

    }


}
