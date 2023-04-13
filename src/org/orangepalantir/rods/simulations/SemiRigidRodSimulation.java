package org.orangepalantir.rods.simulations;

import org.orangepalantir.rods.Motor;
import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.RodViewer;
import org.orangepalantir.rods.Vector;
import org.orangepalantir.rods.integrators.AdaptiveIntegrator;
import org.orangepalantir.rods.integrators.UpdatableAgent;
import org.orangepalantir.rods.interactions.RigidRodAttachment;
import org.orangepalantir.rods.interactions.Spring;
import org.orangepalantir.rods.io.RodIO;

import javax.imageio.ImageIO;
import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.*;

/**
 * Created on 29.03.17.
 */
public class SemiRigidRodSimulation {
    List<RigidRod> actins;
    List<Spring> springs;
    List<Motor> motors;
    ModelConstants constants = new ModelConstants();
    Random ng;
    public void initializeSimulation(Random ng){
        this.ng = ng;
        actins = new ArrayList<>(constants.ACTINS);
        springs = new ArrayList<>();
        motors = new ArrayList<>();
        seedActinFilaments();
        seedCrosslinkers();
        seedMyosinMotors();
    }

    public void placeBoundMotor(Motor motor){
        Map<RigidRod, List<int[]>> possible = new HashMap<>();
        RigidRod r = actins.get(ng.nextInt(actins.size()));
        while(possible.size() == 0) {

            for (RigidRod o : actins) {
                if (o == r) continue;
                List<int[]> ranged = getMyosinBindingSites(r, o);
                if (ranged.size() > 0) {
                    possible.put(o, ranged);
                }
            }
            if(possible.size() == 0){
                r = actins.get(ng.nextInt(actins.size()));
            }
        }
        int n = ng.nextInt(possible.size());
        int c = 0;
        for(var entry: possible.entrySet()){
            if(n == c){
                double[] x1 = new double[3];
                double[] x2 = new double[3];

                RigidRod o = entry.getKey();
                List<int[]> locs = entry.getValue();
                int[] pts = locs.get(ng.nextInt(locs.size()));
                motor.bindRod(r, r.getSFromIndex(pts[0]), 0, constants.MYOSIN_BIND_TIME);
                motor.bindRod(o, o.getSFromIndex(pts[1]), 1, constants.MYOSIN_BIND_TIME);
                r.points[pts[0]].getPosition(x1);
                o.points[pts[1]].getPosition(x2);
                Vector v = new Vector(x2[0] - x1[0], x2[1] - x1[1], x2[2] - x1[2]);
                double[] center = {0.5*(x2[0] + x1[0]), 0.5*(x2[1] + x1[1]),0.5*( x2[2] + x1[2])};
                double pick;
                if(v.length > constants.MYOSIN_LENGTH){
                    pick = 2*constants.MYOSIN_BIND_LENGTH + constants.MYOSIN_LENGTH - v.length;
                } else{
                    pick = constants.MYOSIN_BIND_LENGTH*2 + v.length - constants.MYOSIN_LENGTH;
                }

                x1[0] = center[0] - v.dx*constants.MYOSIN_LENGTH*0.5;
                x1[1] = center[1] - v.dy*constants.MYOSIN_LENGTH*0.5;
                x1[2] = center[2] - v.dz*constants.MYOSIN_LENGTH*0.5 + pick*0.5;

                x2[0] = center[0] + v.dx*constants.MYOSIN_LENGTH*0.5;
                x2[1] = center[1] + v.dy*constants.MYOSIN_LENGTH*0.5;
                x2[2] = center[2] + v.dz*constants.MYOSIN_LENGTH*0.5 + pick*0.5;

                motor.setPosition(0, x1);
                motor.setPosition( 1, x2);
            }
            c++;
        }
    }
    public void seedCrosslinkers(){
        List<RigidRod[]> crosslinkPairs = new ArrayList<>();
        for(int i = 0; i<actins.size(); i++){
            RigidRod a = actins.get(i);
            for(int j = i+1; j<actins.size(); j++){
                RigidRod b = actins.get(j);
                double separation = a.getClosestApproach(b, 200*constants.WIDTH);
                if(separation <= constants.CROSSLINKER_LENGTH){
                    crosslinkPairs.add(new RigidRod[]{a, b});
                }
            }
        }

        for(RigidRod[] rods : crosslinkPairs){
            if(ng.nextDouble() < constants.CROSSLINKER_BIND_FRACTION){
                int[] dexes = rods[0].getIndexesOfClosestApproach(rods[1], 200*constants.WIDTH);
                int d0 = dexes[0] + ng.nextInt(5) - 2;
                if(d0 < 0) d0 = 0;
                if(d0 >= rods[0].points.length) d0 = rods[0].points.length - 1;

                int d1 = dexes[1] + ng.nextInt(5) - 2;
                if(d1 < 0) d1 = 0;
                if(d1 >= rods[1].points.length) d1 = rods[1].points.length - 1;

                double s0 = rods[0].getSFromIndex(d0);
                double s1 = rods[1].getSFromIndex(d1);


                Spring s = new Spring(
                        new RigidRodAttachment(s0, rods[0]),
                        new RigidRodAttachment(s1, rods[1]),
                        constants.WIDTH
                );
                s.setStiffness(constants.SPRING_STIFFNESS);
                s.setRestLength(constants.CROSSLINKER_LENGTH);
                springs.add(s);
            }
        }
        System.out.println(crosslinkPairs.size());
    }

    public RigidRod createNewActinFilament(Point center, Vector direction){
        int N = (int)(constants.ACTIN_LENGTH/constants.S0);
        RigidRod rod = new RigidRod(center, direction, N, constants.ACTIN_LENGTH);
        rod.setStiffness(constants.ACTIN_ELASTIC_MODULOUS);
        rod.setBendingStiffness(constants.ACTIN_BENDING_STIFFNESS);
        return rod;
    }


    public void seedActinFilaments(){
        for(int i = 0; i<constants.ACTINS; i++){
            double seedWidth = 0.25*constants.WIDTH;
            //generate a free filament.
            double x = seedWidth * ng.nextDouble() - 0.5 * seedWidth;
            double y = seedWidth * ng.nextDouble() - 0.5 * seedWidth;
            double z = constants.THICKNESS * ng.nextDouble();// - 0.5 * constants.THICKNESS;

            double theta = 2 * Math.PI * ng.nextDouble();
            double phi = Math.acos(Math.sin(constants.ANGLE_SIGMA/2)*(1 - 2*ng.nextDouble()));


            double nx = Math.cos(theta) * Math.sin(phi);
            double ny = Math.sin(theta) * Math.sin(phi);
            double nz = Math.cos(phi);

            RigidRod rod = createNewActinFilament(new Point(x, y, z), new Vector(nx, ny, nz));

            actins.add(rod);



        }
    }
    public List<int[]> getMyosinBindingSites(RigidRod a, RigidRod b){
        List<int[]> available = new ArrayList<>();
        double[] x1 = new double[3];
        double[] x2 = new double[3];
        double min = constants.MYOSIN_LENGTH - 2*constants.MYOSIN_LENGTH;
        double max = constants.MYOSIN_LENGTH + 2*constants.MYOSIN_BIND_LENGTH;
        for(int i = 0; i<a.points.length; i++){
            Point pa = a.points[i];
            for(int j = 0; j<b.points.length; j++){
                Point pb = b.points[j];
                pa.getPosition(x1);
                pb.getPosition(x2);
                double d = RigidRod.distSqd(x1, x2);
                if(d > min && d < max){
                    available.add(new int[]{i, j});
                }
            }

        }
        return available;
    }
    public void seedMyosinMotors(){
        while(motors.size() < constants.MYOSINS){
            Motor motor = new Motor(
                    constants.MYOSIN_LENGTH,
                    constants.ACTIN_ELASTIC_MODULOUS,
                    constants.MYOSIN_BIND_LENGTH,
                    constants.SPRING_STIFFNESS,
                    constants.MYOSIN_BIND_TIME,
                    constants.WIDTH
            );
            motor.setSteppingForce(constants.MYOSIN_STEPPING_FORCE);
            placeBoundMotor(motor);
            motors.add(motor);
            springs.add(motor.springs[0]);
            springs.add(motor.springs[1]);
        }


    }

    public static void main(String[] args){
        SemiRigidRodSimulation sim = new SemiRigidRodSimulation();
        sim.initializeSimulation(new Random(1));
        RodViewer viewer = new RodViewer();
        sim.actins.forEach(viewer::addRod);
        sim.springs.forEach(viewer::addSpring);
        sim.motors.forEach(viewer::addRod);
        viewer.buildGui();
        viewer.repaint();

        AdaptiveIntegrator integrator = new AdaptiveIntegrator();
        List<UpdatableAgent> agents = new ArrayList<>();
        agents.addAll(sim.actins);
        agents.addAll(sim.motors);
        integrator.prepare(agents);
        int count = 0;
        try(RodIO rio = RodIO.saveRigidRodSimulation() ) {
            rio.setMotors(sim.motors);
            rio.setSprings(sim.springs);
            rio.setRods(sim.actins);
            rio.setWidth(sim.constants.WIDTH);
            rio.setOutput(Paths.get("dats", String.format("initialized-t_%03d.dat", count)));
            rio.write();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        double time = 0;
        long counter = 0;
        while(viewer.displays()){
            double s = integrator.step(sim.springs);
            counter++;
            String status = String.format("relaxed: %2.2f:  %2.4e     %2.4e , %07d",time, integrator.DT, s, counter);
            viewer.setStatus(String.format(status));
            if(counter%50 == 0) {
                count++;
                try(RodIO rio = RodIO.saveRigidRodSimulation() ) {
                    rio.setMotors(sim.motors);
                    rio.setSprings(sim.springs);
                    rio.setRods(sim.actins);
                    rio.setWidth(sim.constants.WIDTH);
                    rio.setOutput(Paths.get("dats", String.format("initialized-t_%03d.dat", count)));
                    rio.write();
                } catch (IOException e) {
                    throw new RuntimeException(e);
                }
                if(count == 200) break;
            };
            viewer.repaint();
            sim.motors.forEach(m -> m.walk(integrator.DT));
        }
    }


}
