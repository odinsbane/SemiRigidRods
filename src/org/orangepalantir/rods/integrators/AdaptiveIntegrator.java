package org.orangepalantir.rods.integrators;

import org.orangepalantir.rods.Motor;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.interactions.Spring;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by msmith on 18/08/16.
 */
public class AdaptiveIntegrator {
    List<double[]> datas = new ArrayList<>();
    List<double[]> forces = new ArrayList<>();
    List<UpdatableAgent> agents;
    public double DT = 1e-3;
    double TOL = 1e-11;

    int positionValues;
    int forcesValues;
    double preparedForces = 0;
    public void prepare(List<? extends UpdatableAgent> agents){
        this.agents = new ArrayList<>(agents);
        int pTally = 0;
        int fTally = 0;

        for(UpdatableAgent agent: agents){
            pTally += agent.getValueCount();
            fTally += agent.getForceCount();
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

    public void storePositions(int slot){
        double[] data = datas.get(slot);
        int offset = 0;
        for(UpdatableAgent agent: agents){
            agent.storePositions(data, offset);
            offset += agent.getValueCount();
        }
    }

    public void storeForces(int slot){
        double[] data = forces.get(slot);
        int offset = 0;
        for(UpdatableAgent agent: agents){
            agent.storeForces(data, offset);
            offset += agent.getForceCount();
        }
    }

    public void restorePositions(int slot){
        double[] data = datas.get(slot);
        int offset = 0;
        for(UpdatableAgent agent: agents){
            agent.restorePositions(data, offset);
            offset += agent.getValueCount();
        }
    }

    public void restoreForces(int slot){
        double[] data = forces.get(slot);
        int offset = 0;
        for(UpdatableAgent agent: agents){
            agent.restoreForces(data, offset);
            offset += agent.getForceCount();
        }

    }

    public double prepareInternalForces(){
        double sum = 0;
        for(UpdatableAgent agent: agents){
            sum += agent.prepareInternalForces();
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
    public void clearForces(){
        agents.forEach(a->a.clearForces());
    }

    public void stepRods(double dt){
        agents.forEach(r->r.step(dt));
    }

    public double step(List<Spring> springs){
        double error;
        do {
            clearForces();
            //store the original positions.
            storePositions(0);
            //apply external forces.
            applyForces(springs);
            //prepare the internal forces.
            preparedForces = prepareInternalForces();
            //store forces
            storeForces(0);
            //take full step
            stepRods(DT);
            storePositions(1);
            //restore 1st positions & 1st forces
            restorePositions(0);
            restoreForces(0);
            //take half-step
            stepRods(DT / 2);
            //store positions
            storePositions(2);

            //re-apply external forces.
            clearForces();
            applyForces(springs);

            //prepare internal forces & store new force state.
            prepareInternalForces();
            storeForces(1);

            //take second half step.
            stepRods(DT / 2);
            storePositions(3);
            //compare error:
            error = calculatePositionError(1, 3);
            // - accepted? adjust dt and forget everything.
            if (error < 1.5*TOL) {
                double f = Math.sqrt(TOL/error);
                if(f>1.2){
                    f=1.2;
                }
                DT = f*DT;
                if(DT>1e-3){
                    DT = 1e-3;
                }
                clearForces();
                applyForces(springs);
                return prepareInternalForces();
            } else{
                restorePositions(0);
                restoreForces(0);
                DT=DT/2;
            }
        } while(error>=1.5*TOL);
        // - rejected? adjust dt to 1/2
        return preparedForces;
    }

    public static void main(String[] args){

    }

}

