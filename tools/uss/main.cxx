#include <stdio.h>
#include <unistd.h>

#include <vector>

#include "uss.h"

using namespace std;
using namespace sensor::uss;


int main(int argc, char *argv[])
{
  JennyUss        uss;
  vector<double>  m;

  fprintf(stderr, "sleeping for 2 seconds\n");
  usleep(2000000);

  uss.open();

  while( true )
  {
    usleep(50000);
    uss.readSensors();

    uss.getUssData(m);

    for(size_t i=0; i<m.size(); ++i)
    {
      fprintf(stderr, "%lf ", m[i]);
    }
    fprintf(stderr, "\n");
  }

  return 0;
}

