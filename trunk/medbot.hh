#include "stage.hh"

using namespace Stg;

#ifndef __MEDBOT_H
#define __MEDBOT_H

const bool verbose = false;

// navigation control params
const double cruisespeed = 0.4;
const double avoidspeed = 0.05;
const double avoidturn = 0.5;
const double minfrontdistance = 0.7;
const double stopdist = 0.5;
const int avoidduration = 10;
const int workduration = 20;
const int payload = 1;

/*
const double cruisespeed = 0.25;
const double avoidspeed = 0.02;
const double avoidturn = 0.1;               // Speed

const double minfrontdistance = 0.75;
const double stopdist = 0.65;                // Distance
*/

double refuel[4][4] =
{
    { -120, -180, 180, 180 },
    { -90, -120, 180, 90 },
    { 0, -90, 180, 180 },
    { -45, 45, -120, -90 }
};

double hospital[4][4] =
{
    {  0, 0, 45, 120 },
    { 0,-90, -60, -160 },
    { -90, -90, 180, 180 },
    { -90, -180, -90, -90 }
};

typedef enum
{
    MODE_IDLE=0,
    MODE_DOCK,
    MODE_UNDOCK,
    MODE_DEAD,
    MODE_RESCUE,
    MODE_APPROCHE,
    MODE_LOAD,
    MODE_UNLOAD,
    MODE_HOSPITAL
} nav_mode_t;

GQueue *jobs;
GHashTable *robots_table;

struct Job
{
    ModelPosition *patient;
    stg_usec_t timestamp;
    void *assignedTo;

    Job(ModelPosition *pos, void *agent)
    {
        patient = pos;
        timestamp = pos->GetWorld()->SimTimeNow();
        assignedTo = agent;
    }
};

#endif
