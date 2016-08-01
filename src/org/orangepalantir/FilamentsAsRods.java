package org.orangepalantir;

import org.paluchlab.agentcortex.CortexModel;
import org.paluchlab.agentcortex.agents.ActinFilament;
import org.paluchlab.agentcortex.io.SimulationReader;

import java.awt.EventQueue;
import java.io.File;

/**
 * Created by melkor on 7/31/16.
 */
public class FilamentsAsRods {

    public static void main(String[] args) throws InterruptedException {
        SimulationReader reader = SimulationReader.fromLockFile(new File("./lf200-first.lock"));
        CortexModel model = reader.model;
        model.setTimePoint(reader.getTimePoint(reader.getPointCount()-1));
        model.prepareForces();

        RodViewer viewer = new RodViewer();
        EventQueue.invokeLater(viewer::buildGui);
        for(ActinFilament actin: model.getActin()) {
            double[] v = actin.direction;
            RigidRod rod = new RigidRod(new Point(0, 0, 0), new Vector(v[0], v[1], 0), 21, actin.length);
            viewer.addRod(rod);
            for (double[] force : actin.forces) {
                rod.applyForce(force[0], force[1], force[2], force[3]);
            }
            double s = rod.relax();
            while(s>1e-2){
                s = rod.relax();
            }
            viewer.repaint();


        }






    }
}
