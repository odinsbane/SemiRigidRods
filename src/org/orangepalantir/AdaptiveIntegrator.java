package org.orangepalantir;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by msmith on 18/08/16.
 */
public class AdaptiveIntegrator {
    List<double[]> datas = new ArrayList<>();
    List<double[]> forces = new ArrayList<>();
    double DT = 1e-3;

    int positionValues;
    int forcesValues;

    public void prepare(List<RigidRod> rods){
        int pTally = 0;
        int fTally = 0;

        for(RigidRod rod: rods){
            pTally += rod.getValueCount();
            fTally += rod.getForceCount();
        }

        positionValues = pTally;
        forcesValues = fTally;
        datas.add(new double[positionValues]);
        datas.add(new double[positionValues]);
        datas.add(new double[positionValues]);
        datas.add(new double[positionValues]);
        forces.add(new double[forcesValues]);
        forces.add(new double[forcesValues]);
    }

    public void storePositions(int slot, List<RigidRod> rods){
        double[] data = datas.get(slot);
        int offset = 0;
        for(RigidRod rod: rods){
            rod.storePositions(data, offset);
            offset += rod.getValueCount();
        }
    }

    public void storeForces(int slot, List<RigidRod> rods){
        double[] data = forces.get(slot);
        int offset = 0;
        for(RigidRod rod: rods){
            rod.storeForces(data, offset);
            offset += rod.getForceCount();
        }
    }

    public void restorePositions(int slot, List<RigidRod> rods){
        double[] data = datas.get(slot);
        int offset = 0;
        for(RigidRod rod: rods){
            rod.restorePositions(data, offset);
            offset += rod.getValueCount();
        }
    }

    public void restoreForces(int slot, List<RigidRod> rods){
        double[] data = forces.get(slot);
        int offset = 0;
        for(RigidRod rod: rods){
            rod.restoreForces(data, offset);
            offset += rod.getForceCount();
        }

    }

    public double prepareInternalForces(List<RigidRod> rods){
        double sum = 0;
        for(RigidRod rod: rods){
            sum += rod.prepareInternalForces();
        }
        return sum;
    }

    public void applyForces(List<Spring> springs){
        for(Spring spring: springs){
            spring.applyForces();
        }
    }

    public double calculatePositionError(int slotA, int slotB){
        double[] dataA = datas.get(slotA);
        double[] dataB = datas.get(slotB);
        double v;
        double sum = 0;
        for(int i = 0; i<dataA.length; i++){
            v = dataA[i] - dataB[i];
            sum += v*v;
        }
        return Math.sqrt(sum);
    }
    public void clearForces(List<RigidRod> rods){
        rods.forEach(RigidRod::clearForces);
    }

    public void stepRods(double dt, List<RigidRod> rods){
        rods.forEach(r->r.step(dt));
    }

    public double step(List<RigidRod> rods, List<Spring> springs){
        //store the original positions.
        storePositions(0, rods);
        //apply external forces.
        applyForces(springs);
        //prepare the internal forces.
        prepareInternalForces(rods);
        //store forces
        storeForces(0, rods);
        //take full step
        stepRods(DT, rods);
        storePositions(1, rods);
        //restore 1st positions & 1st forces
        restorePositions(0, rods);
        restoreForces(0, rods);
        //take half-step
        stepRods(DT/2, rods);
        //store positions
        storePositions(2, rods);

        //re-apply external forces.
        clearForces(rods);
        applyForces(springs);

        //prepare internal forces & store new force state.
        prepareInternalForces(rods);
        storeForces(1, rods);

        //take second half step.
        stepRods(DT/2, rods);
        storePositions(3, rods);
        //compare error:
        double error = calculatePositionError(1, 3);
        // - accepted? adjust dt and forget everything.

        // - rejected? ajust dt to 1/2
        return 0;
    }

    public static void main(String[] args){

    }

}
