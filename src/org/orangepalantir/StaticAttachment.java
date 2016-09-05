package org.orangepalantir;

/**
 * Created by msmith on 05/09/16.
 */
public class StaticAttachment implements Attachment{
    final Point anchor;
    public StaticAttachment(Point a){
        anchor = a;
    }
    @Override
    public Point getAttachment() {
        return anchor;
    }

    @Override
    public void applyForce(Vector v) {
        //pass
    }
}