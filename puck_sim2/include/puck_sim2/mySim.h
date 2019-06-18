#ifndef _MY_SIM_H_
#define _MY_SIM_H_

#include <stdio.h>
#include <cmath>
#include "myPngfile.h"
#include "puck_sim2/puck_sim.h"


#define GRIP_RADIUS (0.05) // in meter
#define PUCK_RADIUS (0.05) // in meter
#define HAND_MAX_V (0.2) // in meter/sec
#define HAND_ACC (0.5) // in time
#define PUCK_DRAG (0.04) // in meter/sec^2

#define MINX (-1.0)
#define MAXX (1.0)
#define MINY (-1.0)
#define MAXY (1.0)

class myCoord {
public:
    double m_x, m_y;
    
    myCoord() {};
    ~myCoord() {};

    double distTo(const myCoord& c)
    {
        double dx, dy;
        dx = this->m_x - c.m_x;
        dy = this->m_y - c.m_y;
        return sqrt(dx*dx+dy*dy);
    }
};

double distToLine(const myCoord& obj, const myCoord& hand_init, const myCoord& hand_fin)
{
    double nx, ny, tx, ty, l, n, t;
    
    tx = hand_fin.m_x - hand_init.m_x;
    ty = hand_fin.m_y - hand_init.m_y;
    l = sqrt(tx*tx+ty*ty);
    tx /= l;
    ty /= l;
    
    nx = -ty;
    ny = tx;
    
    t = (obj.m_x - hand_init.m_x) * tx + (obj.m_y - hand_init.m_y) * ty;
    n = (obj.m_x - hand_init.m_x) * nx + (obj.m_y - hand_init.m_y) * ny;

    if( t < 0.0 || t > l )
        return (sqrt(t*t+n*n));
    else
        return (fabs(n));
}

myCoord collisionCoord(const myCoord& obj, const myCoord& hand_init, const myCoord& hand_fin)
{
    double nx, ny, tx, ty, l, n, t;
    myCoord ret(obj);
    
    tx = hand_fin.m_x - hand_init.m_x;
    ty = hand_fin.m_y - hand_init.m_y;
    l = sqrt(tx*tx+ty*ty);
    tx /= l;
    ty /= l;
    
    nx = -ty;
    ny = tx;
    
    t = (obj.m_x - hand_init.m_x) * tx + (obj.m_y - hand_init.m_y) * ty;
    n = (obj.m_x - hand_init.m_x) * nx + (obj.m_y - hand_init.m_y) * ny;
    
    double dx, dy, rSQ, a, b, c, a1, a2;
    dx = hand_init.m_x - obj.m_x;
    dy = hand_init.m_y - obj.m_y;
    rSQ = (GRIP_RADIUS+PUCK_RADIUS)*(GRIP_RADIUS+PUCK_RADIUS);
    b = (tx*dx+ty*dy);
    c = dx*dx + dy*dy - rSQ;
    a1 = (-sqrt(b*b-c) - b);
    a2 = (sqrt(b*b-c) - b);
    if( a1 < 0.0 )
        a = a2;
    else
        a = a1;

    ret.m_x = hand_init.m_x + a * tx;
    ret.m_y = hand_init.m_y + a * ty;
    
    return (ret);
}

double collisionVel(const myCoord& init, const myCoord& middle, const myCoord& fin)
{
    double tx, ty, length, d;

    tx = fin.m_x - init.m_x;
    ty = fin.m_y - init.m_y;
    length = sqrt(tx*tx+ty*ty);
    
    tx = middle.m_x - init.m_x;
    ty = middle.m_y - init.m_y;
    d = sqrt(tx*tx+ty*ty);

    double distAcc = HAND_MAX_V * HAND_ACC;
    double t, v;
    if( length < distAcc ) // triangle profile
    {
        if( d < 0.5*length )
        {
            t = sqrt(d*HAND_ACC/(2.0*HAND_MAX_V));
            v = HAND_MAX_V / HAND_ACC * t;
        }
        else
        {
            d = length - d;
            t = sqrt(d*HAND_ACC/(2.0*HAND_MAX_V));
            v = HAND_MAX_V / HAND_ACC * t;
        }
    }
    else 
    {
        if( d < 0.5 * HAND_MAX_V * HAND_ACC )
        {
            t = sqrt(d*HAND_ACC/(2.0*HAND_MAX_V));
            v = HAND_MAX_V / HAND_ACC * t;
        }
        else if( length - d < 0.5 * HAND_MAX_V * HAND_ACC )
        {
            d = length - d;
            t = sqrt(d*HAND_ACC/(2.0*HAND_MAX_V));
            v = HAND_MAX_V / HAND_ACC * t;
        }
        else
        {
            v = HAND_MAX_V;
        }
    }
    return (v);
}

class mySim
{
public:
    mySim() {};
    ~mySim() {};

    myCoord run(const myCoord& obj_init, const myCoord& hand_init, const myCoord& hand_fin, const char *filename = NULL)
    {
        double dist;
        
        dist = distToLine(obj_init, hand_init, hand_fin);
        if( dist > GRIP_RADIUS + PUCK_RADIUS )
            return (obj_init);
        
        myCoord contact = collisionCoord(obj_init, hand_init, hand_fin);
        double v = collisionVel(hand_init, contact, hand_fin);
        
        double nx, ny, tx, ty, l, vn;
        nx = obj_init.m_x - contact.m_x;
        ny = obj_init.m_y - contact.m_y;
        l = sqrt(nx*nx+ny*ny);
        nx /= l;
        ny /= l;
        tx = hand_fin.m_x - hand_init.m_x;
        ty = hand_fin.m_y - hand_init.m_y;
        l = sqrt(tx*tx+ty*ty);
        tx /= l;
        ty /= l;
        vn = v*(nx*tx+ny*ty) * 1.0;
        
        double time, d;
        time = vn / PUCK_DRAG; // time to stop
        d = 0.5 * vn * time;
        
        myCoord ret;
        ret.m_x = obj_init.m_x + d * nx;
        ret.m_y = obj_init.m_y + d * ny;
        
        if( filename )
        {
//            myPngfile file(filename);
//            file.setSize(500,500,MINX, MINY,MAXX,MAXY);
//            file.setBackGroundColor('w');
//            file.addCircle(hand_init.m_x,hand_init.m_y,GRIP_RADIUS,0.0,'k');
//            file.addCircle(contact.m_x,contact.m_y,GRIP_RADIUS,0.0,'r');
//            file.addCircle(hand_fin.m_x,hand_fin.m_y,GRIP_RADIUS,0.0,'y');
//            file.addCircle(obj_init.m_x,obj_init.m_y,PUCK_RADIUS,0.0,'b');
//            file.addCircle(ret.m_x,ret.m_y,PUCK_RADIUS,0.0,'g');
//            file.save();
            ROS_INFO("HAND INIT : [%.2f, %.2f].", hand_init.m_x, hand_init.m_y);
            ROS_INFO("CONTACT   : [%.2f, %.2f].", contact.m_x, contact.m_y);
            ROS_INFO("HAND FIN  : [%.2f, %.2f].", hand_fin.m_x, hand_fin.m_y);
            ROS_INFO("OBJ INIT  : [%.2f, %.2f].", obj_init.m_x, obj_init.m_y);
            ROS_INFO("OBJ FIN   : [%.2f, %.2f].", ret.m_x, ret.m_y);
        }

        return (ret);
    }
};

#endif // _MY_ROBOTS_H_
