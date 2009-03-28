#include "stage.hh"
#include "medbot.hh"

using namespace Stg;

class MedRobot
{
private:
    ModelPosition* pos;
    ModelLaser* laser;
    ModelRanger* ranger;
    ModelFiducial* fiducial;
    ModelBlobfinder* blobfinder;
    ModelGripper* gripper;
    Model *source, *sink;
    int avoidcount, randcount;
    int work_get, work_put;
    bool charger_ahoy;
    double charger_bearing;
    double charger_range;
    double charger_heading;
    nav_mode_t mode;
    bool at_dest;

    Job* myjob;

    bool patient_ahead;
    double patient_bearing;
    double patient_range;
    double patient_heading;

public:

    int parked;

    ModelPosition* getPos()
    {
        return pos;
    }

    uint32_t GetId(){
        return pos->GetId();
    }

    MedRobot( ModelPosition* pos )
            : pos(pos),
            laser( (ModelLaser*)pos->GetUnusedModelOfType( MODEL_TYPE_LASER )),
            ranger( (ModelRanger*)pos->GetUnusedModelOfType( MODEL_TYPE_RANGER )),
            fiducial( (ModelFiducial*)pos->GetUnusedModelOfType( MODEL_TYPE_FIDUCIAL )),
            blobfinder( (ModelBlobfinder*)pos->GetUnusedModelOfType( MODEL_TYPE_BLOBFINDER )),
            gripper( (ModelGripper*)pos->GetUnusedModelOfType( MODEL_TYPE_GRIPPER )),
            avoidcount(0),
            randcount(0),
            work_get(0),
            work_put(0),
            charger_ahoy(false),
            charger_bearing(0),
            charger_range(0),
            charger_heading(0),
            mode(MODE_IDLE),
            patient_ahead( false ),
            patient_bearing(0),
            patient_range(0),
            patient_heading(0)
    {
        // need at least these models to get any work done
        // (pos must be good, as we used it in the initialization list)
        assert( laser );

        // PositionUpdate() checks to see if we reached source or sink
        pos->AddUpdateCallback( (stg_model_callback_t)PositionUpdate, this );
        pos->Subscribe();

        // LaserUpdate() controls the robot, by reading from laser and
        // writing to position
        laser->AddUpdateCallback( (stg_model_callback_t)LaserUpdate, this );
        laser->Subscribe();

        fiducial->AddUpdateCallback( (stg_model_callback_t)FiducialUpdate, this );
        fiducial->Subscribe();

        //gripper->AddUpdateCallback( (stg_model_callback_t)GripperUpdate, this );
        gripper->Subscribe();

        if ( blobfinder ) // optional
        {
            blobfinder->AddUpdateCallback( (stg_model_callback_t)BlobFinderUpdate, this );
            blobfinder->Subscribe();
        }

        if (pos->GetWorld()->robots_table == NULL)
            pos->GetWorld()->robots_table = g_hash_table_new( g_direct_hash, g_direct_equal );

        if (pos->GetWorld()->jobs == NULL)
            pos->GetWorld()->jobs =  g_queue_new();

        robots_table = pos->GetWorld()->robots_table;
        jobs = pos->GetWorld()->jobs;

        parked = false;
        myjob = NULL;
    }

    void Approche()
    {
        // close the grippers so they can be pushed into the charger
        ModelGripper::config_t gripper_data = gripper->GetConfig();

        if ( gripper_data.paddles != ModelGripper::PADDLE_CLOSED )
            gripper->CommandClose();
        else  if ( gripper_data.lift != ModelGripper::LIFT_UP )
            gripper->CommandUp();

        if ( patient_ahead )
        {
            double a_goal = normalize( patient_bearing );

            if ( patient_range > 1 )
            {
                if ( !ObstacleAvoid() )
                {
                    pos->SetXSpeed( cruisespeed );
                    pos->SetTurnSpeed( a_goal );
                }
            }
            else
            {
                pos->SetTurnSpeed( a_goal );
                pos->SetXSpeed( 0.02 );	// creep towards it

                if ( patient_range < 0.26 ) // close enough
                    pos->Stop();

                printf("range = %f \n", patient_range);

                if ( pos->Stalled() ) // touching
                    pos->SetXSpeed( -0.01 ); // back off a bit

            }
        }
        else
        {
            //printf( "docking but can't see a charger\n" );
            pos->Stop();
            mode = MODE_RESCUE;
        }

        // if the battery is charged, go back to work
        if ( Full() )
        {
            //printf( "fully charged, now back to work\n" );
            mode = MODE_UNDOCK;
        }
    }

    void Dock()
    {
        // close the grippers so they can be pushed into the charger
        ModelGripper::config_t gripper_data = gripper->GetConfig();

        if ( gripper_data.paddles != ModelGripper::PADDLE_CLOSED )
            gripper->CommandClose();
        else  if ( gripper_data.lift != ModelGripper::LIFT_UP )
            gripper->CommandUp();

        if ( charger_ahoy )
        {
            double a_goal = normalize( charger_bearing );

            // 		if( pos->Stalled() )
            //  		  {
            // 			 puts( "stalled. stopping" );
            //  			 pos->Stop();
            //		  }
            // 		else

            if ( charger_range > 0.5 )
            {
                if ( !ObstacleAvoid() )
                {
                    pos->SetXSpeed( cruisespeed );
                    pos->SetTurnSpeed( a_goal );
                }
            }
            else
            {
                pos->SetTurnSpeed( a_goal );
                pos->SetXSpeed( 0.02 );	// creep towards it

                if ( charger_range < 0.08 ) // close enough
                    pos->Stop();

                if ( pos->Stalled() ) // touching
                    pos->SetXSpeed( -0.01 ); // back off a bit

            }
        }
        else
        {
            //printf( "docking but can't see a charger\n" );
            pos->Stop();
            mode = MODE_IDLE;
        }

        // if the battery is charged, go back to work
        if ( Full() )
        {
            //printf( "fully charged, now back to work\n" );
            mode = MODE_UNDOCK;
        }
    }


    void UnDock()
    {
        const stg_meters_t gripper_distance = 0.2;
        const stg_meters_t back_off_distance = 1.0;
        const stg_meters_t back_off_speed = -0.01;

        // back up a bit
        if ( charger_range < back_off_distance )
            pos->SetXSpeed( back_off_speed );
        else
            pos->SetXSpeed( 0.0 );

        // once we have backed off a bit, open and lower the gripper
        ModelGripper::config_t gripper_data = gripper->GetConfig();
        if ( charger_range > gripper_distance )
        {
            if ( gripper_data.paddles != ModelGripper::PADDLE_OPEN )
                gripper->CommandOpen();
            else if ( gripper_data.lift != ModelGripper::LIFT_DOWN )
                gripper->CommandDown();
        }

        // if the gripper is down and open and we're away from the charger, undock is finished
        if ( gripper_data.paddles == ModelGripper::PADDLE_OPEN &&
                gripper_data.lift == ModelGripper::LIFT_DOWN &&
                charger_range > back_off_distance )
            mode = MODE_IDLE;
    }

    bool ObstacleAvoid()
    {
        bool obstruction = false;
        bool stop = false;

        // find the closest distance to the left and right and check if
        // there's anything in front
        double minleft = 1e6;
        double minright = 1e6;

        // Get the data
        uint32_t sample_count=0;
        stg_laser_sample_t* scan = laser->GetSamples( &sample_count );

        for (uint32_t i = 0; i < sample_count; i++)
        {
            if ( verbose ) printf( "%.3f ", scan[i].range );

            if ( (i > (sample_count/4))
                    && (i < (sample_count - (sample_count/4)))
                    && scan[i].range < minfrontdistance)
            {
                if ( verbose ) puts( "  obstruction!" );
                obstruction = true;
            }

            if ( scan[i].range < stopdist )
            {
                if ( verbose ) puts( "  stopping!" );
                stop = true;
            }

            if ( i > sample_count/2 )
                minleft = MIN( minleft, scan[i].range );
            else
                minright = MIN( minright, scan[i].range );
        }

        if ( verbose )
        {
            puts( "" );
            printf( "minleft %.3f \n", minleft );
            printf( "minright %.3f\n ", minright );
        }

        if ( obstruction || stop || (avoidcount>0) )
        {
            if ( verbose ) printf( "Avoid %d\n", avoidcount );

            pos->SetXSpeed( stop ? 0.0 : avoidspeed );

            /* once we start avoiding, select a turn direction and stick
                with it for a few iterations */
            if ( avoidcount < 1 )
            {
                if ( verbose ) puts( "Avoid START" );
                avoidcount = random() % avoidduration + avoidduration;

                if ( minleft < minright  )
                {
                    pos->SetTurnSpeed( -avoidturn );
                    if ( verbose ) printf( "turning right %.2f\n", -avoidturn );
                }
                else
                {
                    pos->SetTurnSpeed( +avoidturn );
                    if ( verbose ) printf( "turning left %2f\n", +avoidturn );
                }
            }

            avoidcount--;

            return true; // busy avoding obstacles
        }

        return false; // didn't have to avoid anything
    }

    void Die()
    {
        pos->Stop();

        if ( ! Dead() )
            mode = MODE_IDLE;
    }

    void Idle()
    {
        if ( Dead() )
        {
            mode = MODE_DEAD;
            return;
        }

        if ( ! ObstacleAvoid() )
        {
            if ( verbose ) puts( "Cruise" );

            ModelGripper::config_t gdata = gripper->GetConfig();

            //avoidcount = 0;
            pos->SetXSpeed( cruisespeed );

            Pose pose = pos->GetPose();

            int x = (pose.x + 8) / 4;
            int y = (pose.y + 8) / 4;

            // oh what an awful bug - 5 hours to track this down. When using
            // this controller in a world larger than 8*8 meters, a_goal can
            // sometimes be NAN. Causing trouble WAY upstream.
            if ( x > 3 ) x = 3;
            if ( y > 3 ) y = 3;
            if ( x < 0 ) x = 0;
            if ( y < 0 ) y = 0;

			// Always be around Med-Charging station
			double a_goal = dtor( refuel[y][x] );

            // if we are low on juice - find the direction to the recharger instead
            if ( Hungry() )
            {
                //puts( "hungry - using refuel map" );

                // use the refuel map
                a_goal = dtor( refuel[y][x] );

                if ( charger_ahoy ) // I see a charger while hungry!
                    mode = MODE_DOCK;

                parked = false;
            }
            else
            {
                if ( ! at_dest )
                {
                    if ( gdata.beam[0] ) // inner break beam broken
                        gripper->CommandClose();
                }

                // Check for dead robots
                for ( GList *current = g_hash_table_get_values (robots_table); current; current = g_list_next(current))
                {
                    const char* robot_token = (const char*)current->data;
                    ModelPosition* bot =  (ModelPosition*) pos->GetWorld()->GetModel( robot_token );

                    // Check if dead
                    if(bot->FindPowerPack()->ProportionRemaining() < 0.10 )
                    {

                        // Check if it is already on the job list
                        if(!findJob(jobs, bot))
                        {
                            // Create the job
                            Job* j = new Job(bot, NULL);

                            // Add it to global job queue
                            g_queue_push_tail (jobs, j);

                            // If I don't have a job assign it to me
                            if(!myjob){
                                myjob = j;
                                j->assignedTo = this;

                                //if ( verbose )
                                    printf("\nRobot (%d) died. I (%d) am going to help it! \n", bot->GetId(), pos->GetId());
                            }
                        }
                    }
                }

                // Todo: Pick up the next slow job
                if( !myjob && !g_queue_is_empty (jobs) )
                {

                }

                if ( myjob )
                {
                    // I've got work to do
                    mode = MODE_RESCUE;
                    return;
                }
            }

            assert( ! isnan(a_goal ) );
            assert( ! isnan(pose.a ) );

            double a_error = normalize( a_goal - pose.a );

            assert( ! isnan(a_error) );

			if ( charger_ahoy || parked )
			{
			    if( charger_range < 2)
			    {
                    pos->Stop();
                    parked = true;
			    }
			}
            else
            {
                pos->SetTurnSpeed(  a_error );
            }
        }
    }

    Job* findJob(GQueue *jobs_queue, ModelPosition* modPos)
    {
        GList *list = g_queue_peek_head_link(jobs_queue);

        for (GList *current = list; current; current = g_list_next(current))
        {
            Job* job = (Job *)current->data;
            if (job->location == modPos)
                return job;
        }

        return NULL;
    }

    void Rescue()
    {
        if ( ! ObstacleAvoid() )
        {
            if ( Hungry() )
            {
                mode = MODE_IDLE;
                return;
            }

            if ( patient_ahead )
                mode = MODE_APPROCHE;

            pos->SetXSpeed( cruisespeed );

            Pose pose = pos->GetPose();

            Pose dead_body = myjob->location->GetPose();

            double x1 = pose.x + 8;
            double y1 = pose.y + 8;
            double x2 = dead_body.x + 8;
            double y2 = dead_body.y + 8;

            //printf("x1 = %d, y1 = %d    -    x2 = %d, y2 = %d\n", (int)x1, (int)y1, (int)x2, (int)y2);

            double a_goal = -atan2(x1 - x2, y1 - y2) - (dtor(90));

            printf("a_goal = %f \n", rtod(a_goal));

            double a_error = normalize( a_goal - pose.a );

            assert( ! isnan(a_error) );

            pos->SetTurnSpeed(  a_error );

            //pos->SetTurnSpeed(0);
            //pos->SetXSpeed(0);
        }
    }

    // inspect the laser data and decide what to do
    static int LaserUpdate( ModelLaser* laser, MedRobot* robot )
    {
        //   if( laser->power_pack && laser->power_pack->charging )
        // 	 printf( "model %s power pack @%p is charging\n",
        // 				laser->Token(), laser->power_pack );

        if ( laser->GetSamples(NULL) == NULL )
            return 0;

        switch ( robot->mode )
        {
            case MODE_DOCK:
                //puts( "DOCK" );
                robot->Dock();
                break;

            case MODE_UNDOCK:
                //puts( "UNDOCK" );
                robot->UnDock();
                break;

            case MODE_IDLE:
                //puts( "IDLE" );
                robot->Idle();
                break;

            case MODE_DEAD:
                //puts( "DEAD" );
                robot->Die();
                break;

            case MODE_RESCUE:
                robot->Rescue();
                break;

            case MODE_APPROCHE:
                robot->Approche();
                break;

            default:
                printf( "unrecognized mode %u\n", robot->mode );
        }

        return 0;
    }

    bool Dead()
    {
        return( pos->FindPowerPack()->ProportionRemaining() < 0.05 );
    }

    bool Hungry()
    {
        return( pos->FindPowerPack()->ProportionRemaining() < 0.40 );
    }

    bool Full()
    {
        return( pos->FindPowerPack()->ProportionRemaining() > 0.95 );
    }

    static int PositionUpdate( ModelPosition* pos, MedRobot* robot )
    {
        Pose pose = pos->GetPose();

        //printf( "Pose: [%.2f %.2f %.2f %.2f]\n",
        //  pose.x, pose.y, pose.z, pose.a );

        //pose.z += 0.0001;
        //robot->pos->SetPose( pose );

        if ( pos->GetFlagCount() < payload &&
                hypot( -7-pose.x, -7-pose.y ) < 2.0 )
        {
            if ( ++robot->work_get > workduration )
            {
                // protect source from concurrent access
                //robot->source->Lock();

                // transfer a chunk from source to robot
                //pos->PushFlag( robot->source->PopFlag() );
                //robot->source->Unlock();

                //robot->work_get = 0;
            }
        }

        robot->at_dest = false;

        if ( hypot( 7-pose.x, 7-pose.y ) < 1.0 )
        {
            robot->at_dest = true;

            robot->gripper->CommandOpen();

            if ( ++robot->work_put > workduration )
            {
                // protect sink from concurrent access
                //robot->sink->Lock();

                //puts( "dropping" );
                // transfer a chunk between robot and goal
                //robot->sink->PushFlag( pos->PopFlag() );
                //robot->sink->Unlock();

                robot->work_put = 0;

            }
        }


        return 0; // run again
    }



    static int FiducialUpdate( ModelFiducial* mod, MedRobot* robot )
    {
        robot->charger_ahoy = false;

        for ( unsigned int i = 0; i < mod->fiducial_count; i++ )
        {
            stg_fiducial_t* f = &mod->fiducials[i];

            //printf( "fiducial %d is %d at %.2f m %.2f radians\n",
            //	  i, f->id, f->range, f->bearing );

            if ( f->id == 9110 ) // I see a charging station
            {
                // record that I've seen it and where it is
                robot->charger_ahoy = true;
                robot->charger_bearing = f->bearing;
                robot->charger_range = f->range;
                robot->charger_heading = f->geom.a;

                //printf( "charger at %.2f radians\n", robot->charger_bearing );
                break;
            }

            if ( f->id == (int)robot->myjob->location->GetId())   // I see the dead robot
            {
                robot->patient_ahead = true;
                robot->patient_bearing = f->bearing;
                robot->patient_range = f->range;
                robot->patient_heading = f->geom.a;
            }
        }

        return 0; // run again
    }

    static int BlobFinderUpdate( ModelBlobfinder* blobmod, MedRobot* robot )
    {
        unsigned int blob_count = 0;
        stg_blobfinder_blob_t* blobs = blobmod->GetBlobs( &blob_count );

        if ( blobs && (blob_count>0) )
        {
            printf( "%s sees %u blobs\n", blobmod->Token(), blob_count );
        }

        return 0;
    }

    static int GripperUpdate( ModelGripper* grip, MedRobot* robot )
    {
        ModelGripper::config_t gdata = grip->GetConfig();

        printf( "BREAKBEAMS %s %s\n",
                gdata.beam[0] ? gdata.beam[0]->Token() : "<null>",
                gdata.beam[1] ? gdata.beam[1]->Token() : "<null>" );

        printf( "CONTACTS %s %s\n",
                gdata.contact[0] ? gdata.contact[0]->Token() : "<null>",
                gdata.contact[1] ? gdata.contact[1]->Token() : "<null>");


        return 0;
    }

};

// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
    new MedRobot( (ModelPosition*)mod);

    return 0; //ok
}



