#include "stage.hh"
using namespace Stg;

const int INTERVAL = 200;
const double FLAGSZ = 0.4;

int Update( Model* mod, void* dummy );

const int STAT_INTERVAL = 1000000 * 60;

FILE * pFile;

// Stage calls this when the model starts up
extern "C" int Init( Model* mod )
{
  mod->AddUpdateCallback( (stg_model_callback_t)Update, NULL );

  pFile = fopen ("source_stat_1.csv","w");
  fprintf(pFile, "Number of Flags,Timestamp (m)\n");

  return 0; //ok
}

// inspect the laser data and decide what to do
int Update( Model* mod, void* dummy )
{
  if( mod->GetWorld()->GetUpdateCount() % INTERVAL  == 0  && mod->GetFlagCount() < 10){
	 mod->PushFlag( new Flag( stg_color_pack( 1,1,0,0), FLAGSZ ) );
  }

  if( mod->GetWorld()->SimTimeNow() % STAT_INTERVAL  == 0 ){
    int c;
    mod->GetWorld()->GetModel("sink")->GetPropertyInt("count", &c, 0);

    if(pFile)
        fprintf(pFile, "%d,%llu\n", c, mod->GetWorld()->SimTimeNow() / STAT_INTERVAL);
  }

  return 0; // run again
}

