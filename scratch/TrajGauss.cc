#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
//#include "ns3/netanim-module.h"
//#include "ns3/buildings-module.h"
#include <fstream>

using namespace ns3;
using namespace std;

const char* fname= "TrajGauss.csv";

void
CourseChange (std::string context, Ptr<const MobilityModel> model)
{
  Vector pos = model->GetPosition ();
  ofstream f;
  f.open(fname,ios_base::app);
  f << pos.x << "," << pos.y << "," << pos.z << endl;
}

int main (int argc, char *argv[])
{
  double simTimeSec=200.0;

  uint32_t numNodes = 1;

  CommandLine cmd;
  cmd.AddValue ("time",  "Simulation time (seconds)", simTimeSec);
  cmd.AddValue ("nodes",  "Number of traffic generating nodes", numNodes);
  cmd.Parse (argc,argv);

  NodeContainer sta;
  sta.Create (numNodes);
  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
				 "X", StringValue ("ns3::UniformRandomVariable[Min=200.0|Max=300.0]"),
				 "Y", StringValue ("ns3::UniformRandomVariable[Min=200.0|Max=300.0]"),
				 "Z", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=200.0]"));

  mobility.SetMobilityModel ("ns3::ObstacleGaussMarkovMobilityModel",
			     "MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=20.0|Max=25.0]"),
			     "MeanPitch", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=3.141592]"),
			     "Alpha", DoubleValue (0.7),
			     "Bounds", BoxValue (Box (0.0, 300.0, 0.0, 300.0, 0.0, 200.0)));

  mobility.Install (sta);

  Ptr<ObstacleGaussMarkovMobilityModel> mob = sta.Get(0)->GetObject<ObstacleGaussMarkovMobilityModel> ();
  mob->AddObstacle(Box(100, 200.0, 100, 200.0, 0.0, 200.0));
  mob->AddObstacle(Box(0.0, 100.0, 0.0, 100.0, 0.0, 100.0));

  ofstream f;
  f.open(fname, ofstream::out | ofstream::trunc);
  f.close();

  Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange",MakeCallback(&CourseChange));

  Simulator::Stop (Seconds (simTimeSec));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
