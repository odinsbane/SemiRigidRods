package org.orangepalantir.rods.interactions;

import org.orangepalantir.rods.Point;
import org.orangepalantir.rods.interactions.Attachment;

/**
 * Created by msmith on 05/09/16.
 */
public class StaticAttachment implements Attachment {
    final Point anchor;
    public StaticAttachment(Point a){
        anchor = a;
    }
    @Override
    public Point getAttachment() {
        return anchor;
    }
}