package org.orangepalantir.rods;

import org.orangepalantir.rods.integrators.UpdatableAgent;
import org.orangepalantir.rods.interactions.Attachment;
import org.orangepalantir.rods.interactions.RigidRodAttachment;
import org.orangepalantir.rods.interactions.Spring;

import java.util.Arrays;
import java.util.List;

/**
 * Final element for creating our simulation. Bind/unbind and apply forces.
 *
 * Created by Matt on 29/09/16.
 */
public class Motor implements DrawableRod, UpdatableAgent{
    public final static int FRONT = 0;
    public final static int BACK = 1;


    public double stalkLength = 0.8;
    public double springLength = 0.2;
    public double springStiffness = 100;
    public double stalkStiffness = 1000;
    double f0 = 1;
    double bindTau = 75;
    double width;
    RigidRodAttachment[] attachments = new RigidRodAttachment[2];
    public Spring[] springs = new Spring[2];
    public double[] bindTimes = new double[2];
    public double[] timeBound = new double[2];
    Point[] points = new Point[2];

    double[] appliedForces = new double[6];
    double[] totalForces = new double[6];
    public Motor(double stalkLength, double stalkStiffness, double springLength, double springStiffness, double bindTau, double width){
        this.stalkLength = stalkLength;
        this.stalkStiffness = stalkStiffness;
        this.springLength = springLength;
        this.springStiffness = springStiffness;
        this.bindTau = bindTau;
        this.width = width;
        points[0] = new Point(0, 0, 0);
        points[1] = new Point(0, 0.8, 0);
        attachments[FRONT] = new RigidRodAttachment(-1, null);
        attachments[BACK] = new RigidRodAttachment(-1, null);
        springs[FRONT] = new Spring(attachments[0], createAttachment(FRONT), Double.MAX_VALUE);
        springs[BACK] = new Spring(attachments[BACK], createAttachment(BACK), Double.MAX_VALUE);
        springs[FRONT].setRestLength(springLength);
        springs[FRONT].setStiffness(springStiffness);
        springs[BACK].setRestLength(springLength);
        springs[BACK].setStiffness(springStiffness);
    }

    public void prepareHeads(){

    }
    public void bindRod(RigidRod rod, double location, int head, double duration){

        bindTimes[head] = duration;
        timeBound[head] = 0;
        attachments[head].setRod(location, rod);
    }

    public RigidRodAttachment getBound(int head){
        return attachments[head];
    }

    Attachment createAttachment(int head){
        return new Attachment(){
            @Override
            public void applyForce(Vector v){
                int dex = 3*head;
                appliedForces[dex] += v.dx*v.length;
                appliedForces[dex+1] += v.dy*v.length;
                appliedForces[dex+2] += v.dz*v.length;
            }

            @Override
            public Point getAttachment() {
                return points[head];
            }
        };
    }
    public void walk(double dt){
        if(springs[0]!=null){
            walk(springs[0], attachments[0], dt);
        }

        if(springs[1]!=null){
            walk(springs[1], attachments[1], dt);
        }

    }

    public boolean isFree(){
        return attachments[0].rod==null && attachments[1].rod==null;
    }

    /**
     * Check if a head has zero stiffness.
     * @return
     */
    public boolean hasFreeHead(){
        if(springs[0]!=null && springs[0].getStiffness()==0) return true;
        if(springs[1]!=null && springs[1].getStiffness()==0) return true;
        return false;
    }

    public void walk(Spring s, RigidRodAttachment a, double dt){
        Vector force = s.getForce();
        Vector tangent = a.rod.getTangent(a.loc);
        double f = force.length*(tangent.dx*force.dx + tangent.dy*force.dy + tangent.dz*force.dz);
        a.loc += (f0 + f)*dt;
        if(a.loc>a.rod.length/2 || a.loc<-a.rod.length/2){
            s.setStiffness(0);
        }
    }

    public void removeSpring(int head){
        springs[head] = null;
    }

    public double prepareInternalForces(){
        System.arraycopy(appliedForces, 0, totalForces, 0, 6);

        Vector stalk = new Vector(points[BACK], points[FRONT]);
        double mag = -stalkStiffness*(stalkLength - stalk.length);
        totalForces[0] += -stalk.dx*mag;
        totalForces[1] += -stalk.dy*mag;
        totalForces[2] += -stalk.dz*mag;
        totalForces[3] += stalk.dx*mag;
        totalForces[4] += stalk.dy*mag;
        totalForces[5] += stalk.dz*mag;

        double sum = 0;
        for(double d: totalForces){
            sum += d*d;
        }
        return Math.sqrt(sum);
    }

    public void restorePositions(double[] data, int offset){
        for(int i = 0; i<points.length; i++){
            int current = i*3 + offset;
            Point p = points[i];
            p.x = data[current];
            p.y = data[current+1];
            p.z = data[current+2];
        }
    }

    @Override
    public void clearForces() {
        Arrays.fill(appliedForces, 0.0);
    }

    @Override
    public void step(double dt) {
        double attempts = 1;
        for(int j = 0; j<attempts; j++){


            points[0].x += totalForces[0] * dt/attempts;
            points[0].y += totalForces[1] * dt/attempts;
            points[0].z += totalForces[2] * dt/attempts;
            points[1].x += totalForces[3] * dt/attempts;
            points[1].y += totalForces[4] * dt/attempts;
            points[1].z += totalForces[5] * dt/attempts;

            if(j<attempts-1) prepareInternalForces();
        }
    }

    public void storePositions(double[] data, int offset){
        for(int i = 0; i<points.length; i++){
            Point p = points[i];
            int current = 3*i + offset;
            data[current] = p.x;
            data[current+1] = p.y;
            data[current+2] = p.z;
        }
    }

    public void storeForces(double[] data, int offset){
        System.arraycopy(totalForces, 0, data, offset, totalForces.length);
    }

    @Override
    public void restoreForces(double[] data, int offset){
        System.arraycopy(data, offset, totalForces, 0, totalForces.length);
    }

    @Override
    public int getValueCount(){
        return 6;
    }

    @Override
    public int getForceCount(){
        return 6;
    }

    @Override
    public List<Point> getPoints() {
        return Arrays.asList(points);
    }

    public double getBindTau() {
        return bindTau;
    }

    public double getTimeRemaining(int head) {
        return bindTimes[head] - timeBound[head];
    }

    public void setPosition(int head, double[] pos) {
        points[head].x = pos[0];
        points[head].y = pos[1];
        points[head].z = pos[2];
    }

    public void setSteppingForce(double myosinSteppingForce) {
        f0 = myosinSteppingForce;
    }
}

