package org.orangepalantir.rods;

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
public class Motor implements DrawableRod{
    final static int FRONT = 0;
    final static int BACK = 1;


    double stalkLength = 0.8;
    double springLength = 0.2;
    double springStiffness = 100;
    double stalkStiffness = 1000;
    double f0 = 1;
    double motorTau = 75;

    double[] bindLocations = new double[2];
    RigidRodAttachment[] attachments = new RigidRodAttachment[2];
    public Spring[] springs = new Spring[2];
    double[] bindTimes = new double[2];
    double[] timeBound = new double[2];
    Point[] points = new Point[2];

    double[] appliedForces = new double[6];
    double[] totalForces = new double[6];
    public Motor(double stalkLength, double stalkStiffness, double springLength, double springStiffness){
        this.stalkLength = stalkLength;
        this.stalkStiffness = 1000;
        points[0] = new Point(0, 0, 0);
        points[1] = new Point(0, 0.8, 0);
    }
    public void bindRod(RigidRod rod, double location, int head, double duration){
        RigidRodAttachment attachment = new RigidRodAttachment(location, rod);
        Spring s = new Spring(
                attachment,
                createAttachment(head)
        );
        s.setRestLength(springLength);
        s.setStiffness(springStiffness);
        bindTimes[head] = duration;
        timeBound[head] = 0;
        attachments[head] = attachment;
        springs[head] = s;
    }

    public void applyForces(){

        if(springs[0]==null || springs[1]==null){
            return;
        }
        springs[0].applyForces();
        springs[1].applyForces();

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
    public void walk(Spring s, RigidRodAttachment a, double dt){
        Vector force = s.getForce();
        Vector tangent = a.rod.getTangent(a.loc);
        double f = force.length*(tangent.dx*force.dx + tangent.dy*force.dy + tangent.dz*force.dz);
        a.loc += (f0 - f)*dt;
    }

    public double prepareInternalForces(){
        System.arraycopy(appliedForces, 0, totalForces, 0, 6);

        Vector stalk = new Vector(points[BACK], points[FRONT]);
        double mag = stalkStiffness*(stalkLength - stalk.length);
        stalk.length = mag;
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

    public void restoreForces(double[] data, int offset){
        System.arraycopy(data, offset, totalForces, 0, totalForces.length);
    }

    public int getValueCount(){
        return 6;
    }

    public int getForceCount(){
        return 6;
    }

    @Override
    public List<Point> getPoints() {
        return Arrays.asList(points);
    }
}

