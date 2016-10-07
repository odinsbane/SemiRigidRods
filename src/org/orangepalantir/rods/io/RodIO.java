package org.orangepalantir.rods.io;

import org.orangepalantir.rods.Motor;
import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.Vector;
import org.orangepalantir.rods.interactions.FixedForceAttachment;
import org.orangepalantir.rods.interactions.RigidRodAttachment;
import org.orangepalantir.rods.interactions.Spring;

import java.io.DataInputStream;
import java.io.DataOutput;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;

/**
 * For loading and saving rods.
 *
 * Created by Matt on 28/09/16.
 */
public class RodIO implements AutoCloseable{
    final static int SAVE = 0;
    final static int LOAD = 1;
    final int mode;
    final static int RIGID_ROD = 0;
    final static int CROSSLINKER = 1;
    final static int MYOSIN = 2;
    final static int FIXED_FORCE=3;
    List<RigidRod> rods;
    List<Spring> springs;
    List<Motor> motors = new ArrayList<>();
    DataOutputStream output;
    DataInputStream input;
    double width = 10;
    long time;
    private RodIO(int mode){
        this.mode = mode;
    }


    private void loadData() throws IOException {
        int read;
        time = input.readLong();
        width = input.readDouble();
        do{
            try {
                read = input.readInt();
            } catch(Exception exc){
                //just return what you have.
                read = -1;
            }
            switch(read){
                case RIGID_ROD:
                    rods.add(readRigidRod());
                    break;
                case CROSSLINKER:
                    springs.add(readCrossLinker());
                    break;
                case MYOSIN:
                    motors.add(readMotor());
                    break;
                case FIXED_FORCE:
                    springs.add(readFixedForce());
                    break;
                default:
                    //oh well.
            }

        }while(read>=0);

    }

    private Spring readCrossLinker() throws IOException {
        double l = input.readDouble();
        double k = input.readDouble();
        int a = input.readInt();
        double as = input.readDouble();
        int b = input.readInt();
        double bs = input.readDouble();
        RigidRodAttachment aa = new RigidRodAttachment(as, rods.get(a));
        RigidRodAttachment ba = new RigidRodAttachment(bs, rods.get(b));
        Spring s = new Spring(aa, ba, width);
        s.setRestLength(l);
        s.setStiffness(k);
        return new Spring(aa, ba, width);
    }

    private Spring readFixedForce() throws IOException {

        FixedForceAttachment ffa = new FixedForceAttachment(
                rods.get(input.readInt()),
                input.readDouble(),
                new double[]{input.readDouble(), input.readDouble(), input.readDouble()}
        );
        return new Spring(ffa, ffa.getDanglingEnd(), width);
    }

    private Motor readMotor() throws IOException {
        double stalkLength = input.readDouble();
        double stalkStiffness = input.readDouble();
        double springLength = input.readDouble();
        double springStiffness = input.readDouble();
        double bindTau = input.readDouble();
        int aDex = input.readInt();
        int bDex = input.readInt();
        double aLoc = input.readDouble();
        double bLoc = input.readDouble();
        double aTime = input.readDouble();
        double bTime = input.readDouble();

        Motor m = new Motor(stalkLength, stalkStiffness, springLength, springStiffness, bindTau, width);
        m.bindRod(rods.get(aDex), aLoc, Motor.FRONT, 0);
        m.bindRod(rods.get(bDex), bLoc, Motor.BACK, -m.getBindTau()*Math.exp(1 - Math.random()));

        return m;

    }

    private void write(Motor motor) throws IOException{
        output.writeInt(MYOSIN);
        output.writeDouble(motor.stalkLength);
        output.writeDouble(motor.stalkStiffness);
        output.writeDouble(motor.springLength);
        output.writeDouble(motor.springStiffness);
        output.writeDouble(motor.getBindTau());
        RigidRodAttachment a = motor.getBound(Motor.FRONT);
        if(a==null){
            output.writeInt(-1);
        }else {
            output.writeInt(rods.indexOf(a.rod));
            output.writeDouble(a.loc);
            output.writeDouble(motor.getTimeRemaining(Motor.FRONT));
        }
        RigidRodAttachment b = motor.getBound(Motor.FRONT);
        if(b==null){
            output.writeInt(-1);
        } else{
            output.writeInt(rods.indexOf(a.rod));
            output.writeDouble(a.loc);
            output.writeDouble(motor.getTimeRemaining(Motor.FRONT));
        }
    }



    private RigidRod readRigidRod() throws IOException {
        double l = input.readDouble();
        double k = input.readDouble();
        double kappa = input.readDouble();
        int N = input.readInt();
        Point[] points = new Point[N];
        for(int i = 0; i<N; i++){
            points[i] = new Point(input.readDouble(), input.readDouble(), input.readDouble());
        }

        return new RigidRod(points, k, kappa, l);
    }

    private void write(RigidRod rod) throws IOException {
        output.writeInt(RIGID_ROD);
        output.writeDouble(rod.length);
        output.writeDouble(rod.k);
        output.writeDouble(rod.kappa);
        output.writeInt(rod.N);
        for(Point p: rod.getPoints()){
            output.writeDouble(p.x);
            output.writeDouble(p.y);
            output.writeDouble(p.z);
        }
    }


    private void write(Spring spring) throws IOException {
        if(spring.a instanceof FixedForceAttachment){
            writeFixedForce(spring);
        } else{
            writeCrosslinker(spring);
        }
    }
    private void writeCrosslinker(Spring s) throws IOException {
        output.writeInt(CROSSLINKER);
        output.writeDouble(s.getRestLength());
        output.writeDouble(s.getStiffness());
        RigidRodAttachment a = (RigidRodAttachment) s.a;
        RigidRodAttachment b = (RigidRodAttachment) s.b;
        output.writeInt(rods.indexOf(a.rod));
        output.writeDouble(a.loc);
        output.writeInt(rods.indexOf(b.rod));
        output.writeDouble(b.loc);

    }

    private void writeFixedForce(Spring s) throws IOException {
        FixedForceAttachment at = (FixedForceAttachment)s.a;
        int dex = rods.indexOf(at.getRod());
        double ap = at.getAttachmentLocation();
        Vector f = s.getForce();
        double fx = f.dx*f.length;
        double fy = f.dy*f.length;
        double fz = f.dz*f.length;
        output.writeInt(FIXED_FORCE);
        output.writeInt(dex);
        output.writeDouble(ap);
        output.writeDouble(fx);
        output.writeDouble(fy);
        output.writeDouble(fz);


    }


    /**
     * For work with a path, uses
     * @param path
     * @throws IOException
     */
    public void setOutput(Path path) throws IOException {
        setOutput(new DataOutputStream(Files.newOutputStream(path, StandardOpenOption.CREATE_NEW)));
    }

    public void setOutput(DataOutputStream out){
        output = out;
    }

    public void setWidth(double w){
        width = w;
    }

    public void write() throws IOException{
        if(mode==LOAD) throw new IOException("RodIO not opened for saving.");

        long time = System.currentTimeMillis();
        output.writeLong(time);
        output.writeDouble(width);
        for(RigidRod rod: rods){
            write(rod);
        }
        for(Motor motor: motors){
            springs.remove(motor.springs[0]);
            springs.remove(motor.springs[1]);
            write(motor);
        }
        for(Spring spring: springs){
            write(spring);
        }
        output.writeInt(-1);
    }

    @Override
    public void close() throws IOException {
        switch(mode){
            case SAVE:
                output.close();
                break;
            case LOAD:
                output.close();
                break;
        }
    }
    public static RodIO saveRigidRodSimulation() throws IOException {

        return new RodIO(SAVE);

    }

    public void setRods(List<RigidRod> rods) {
        this.rods = rods;
    }

    public void setSprings(List<Spring> springs){
        this.springs = springs;
    }

    public static RodIO loadRigidRod(Path path) throws IOException {
        RodIO io = new RodIO(LOAD);
        io.input = new DataInputStream(Files.newInputStream(path));
        io.setRods(new ArrayList<>());
        io.setSprings(new ArrayList<>());
        io.loadData();
        return io;
    }

    public List<RigidRod> getRods() {
        return rods;
    }
}
