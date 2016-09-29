package org.orangepalantir.rods.io;

import org.orangepalantir.rods.Motor;
import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.RigidRod;
import org.orangepalantir.rods.interactions.RigidRodAttachment;
import org.orangepalantir.rods.interactions.Spring;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.util.List;

/**
 * For loading and saving rods.
 *
 * Created by Matt on 28/09/16.
 */
public class RodIO {
    final static int RIGID_ROD = 0;
    final static int CROSSLINKER = 1;
    List<RigidRod> rods;
    List<Spring> springs;
    List<Motor> motors;
    DataOutputStream output;
    DataInputStream input;
    private RodIO(){

    }
    public void loadData() throws IOException {
        int read;
        do{
            read = input.readInt();
            switch(read){
                case RIGID_ROD:
                    rods.add(readRigidRod());
                    break;
                case CROSSLINKER:
                    springs.add(crossLinker());
                    break;
                default:
                    //oh well.
            }

        }while(read>=0);

    }

    public Spring crossLinker() throws IOException {
        double k = input.readDouble();
        double kappa = input.readDouble();
        double l = input.readDouble();
        int a = input.readInt();
        double as = input.readDouble();
        int b = input.readInt();
        double bs = input.readDouble();
        RigidRodAttachment aa = new RigidRodAttachment(as, rods.get(a));
        RigidRodAttachment ba = new RigidRodAttachment(bs, rods.get(b));
        Spring s = new Spring(aa, ba);
        s.setRestLength(l);
        s.setStiffness(k);
        return new Spring(aa, ba);
    }

    public RigidRod readRigidRod() throws IOException {
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

    public void writeRigidRod(RigidRod rod) throws IOException {
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

    public void writeCrosslinkedRods(Spring s) throws IOException {
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

    static RodIO saveCrosslinkedFilaments(List<RigidRod> rods, List<Spring> crosslinkers) throws IOException {
        RodIO io = new RodIO();
        for(RigidRod rod: rods){
            io.writeRigidRod(rod);
        }
        return io;
    }
}
