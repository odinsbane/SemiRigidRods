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
    public double TOL = 1e-4;

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

    public void swapPositions(int i, int j){
        double[] iArray = datas.get(i);
        double[] jArray = datas.get(j);
        datas.set(i, jArray);
        datas.set(j, iArray);
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
        clearForces();
        storePositions(0);
        applyForces(springs);
        preparedForces = prepareInternalForces();
        storeForces(0);
        stepRods(DT);
        storePositions(1);


        do {
            //At the biggining of the loop 0 positions are starting, 0 forces initial forces.
            // 1 position is the full dt step.
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
                if(f>2){
                    f=2;
                }
                DT = f*DT;
                if(DT>1e-3){
                    DT = 1e-3;
                }
                clearForces();
                applyForces(springs);
                return prepareInternalForces();
            } else{
                swapPositions(2, 1);
                DT=DT/2;
            }
        } while(error>=1.5*TOL);
        // - rejected? adjust dt to 1/2
        return preparedForces;
    }

    public static void main(String[] args){

    }

}

