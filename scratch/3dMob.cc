#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include "ns3/applications-module.h"
#include "ns3/propagation-module.h"
#include "ns3/wifi-module.h"
#include "ns3/olsr-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/buildings-module.h"

//#include "ns3/stats-module.h"
#include "ns3/gnuplot.h"

//#include <fstream>

using namespace ns3;
using namespace std;

class Experiment
{
public:
  Experiment ();
  double Run (double time, int model, uint32_t nodes, uint32_t speed, bool withObstacles); // Return control packet pdr
  bool CommandSetup (int argc, char **argv);

  enum Model  {
    WALK_TIME,
    WALK_DISTANCE,
    DIRECTION,
    GAUSS_MARKOV
  };

private:
  void TxOLSR (std::string str, const olsr::PacketHeader &header, const olsr::MessageList &messages);
  void RxOLSR (std::string str, const olsr::PacketHeader &header, const olsr::MessageList &messages);

  uint32_t m_rxCount;
  uint32_t m_txCount;
};

Experiment::Experiment ()
{
  m_rxCount = 0;
  m_txCount = 0;
}

void
Experiment::RxOLSR (std::string str, const olsr::PacketHeader &header, const olsr::MessageList &messages)
{
  m_rxCount += 1;
}
void
Experiment::TxOLSR (std::string str, const olsr::PacketHeader &header, const olsr::MessageList &messages)
{
  m_txCount += 1;
}

double
Experiment::Run(double simtime, int model, uint32_t nodes, uint32_t speed, bool withObstacles)
{
  std::stringstream avgSpeedStr;
  avgSpeedStr << "ns3::ConstantRandomVariable[Constant=" << speed << "]";

  NodeContainer sta;
  NetDeviceContainer devices;
  sta.Create (nodes);

  /*
   * Mobility setup
   */
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
				 "X", StringValue ("ns3::UniformRandomVariable[Min=200.0|Max=300.0]"),
				 "Y", StringValue ("ns3::UniformRandomVariable[Min=200.0|Max=300.0]"),
				 "Z", StringValue ("ns3::UniformRandomVariable[Min=0.01|Max=200.0]"));
  switch (model)
  {
    case Experiment::WALK_TIME:
      mobility.SetMobilityModel ("ns3::RandomWalk3dMobilityModel",
				 "Mode", StringValue ("Time"),
				 "Time", StringValue ("10s"),
				 "Speed", StringValue (avgSpeedStr.str()),
				 "Bounds", BoxValue (Box (0.0, 300.0, 0.0, 300.0, 0.01, 200.0)));
      mobility.Install (sta);
      if (withObstacles)
	{
	  Ptr<RandomWalk3dMobilityModel> mob = sta.Get(0)->GetObject<RandomWalk3dMobilityModel> ();
	  mob->AddObstacle(Box(0.0, 100.0, 0.0, 100.0, 0.0, 100.0));
	  mob->AddObstacle(Box(100, 200.0, 100, 200.0, 0.0, 200.0));
	}
      break;
    case Experiment::WALK_DISTANCE:
      mobility.SetMobilityModel ("ns3::RandomWalk3dMobilityModel",
				 "Mode", StringValue ("Distance"),
				 "Distance",  DoubleValue (200.0),
				 "Speed", StringValue (avgSpeedStr.str()),
				 "Bounds", BoxValue (Box (0.0, 300.0, 0.0, 300.0, 0.01, 200.0)));
      mobility.Install (sta);
      if (withObstacles)
	{
	  Ptr<RandomWalk3dMobilityModel> mob = sta.Get(0)->GetObject<RandomWalk3dMobilityModel> ();
	  mob->AddObstacle(Box(0.0, 100.0, 0.0, 100.0, 0.0, 100.0));
	  mob->AddObstacle(Box(100, 200.0, 100, 200.0, 0.0, 200.0));
	}
      break;
    case Experiment::DIRECTION:
      mobility.SetMobilityModel ("ns3::RandomDirection3dMobilityModel",
				 "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),
				 "Speed", StringValue (avgSpeedStr.str()),
				 "Bounds", BoxValue (Box (0.0, 300.0, 0.0, 300.0, 0.01, 200.0)));
      mobility.Install (sta);
      if (withObstacles)
	{
	  Ptr<RandomDirection3dMobilityModel> mob = sta.Get(0)->GetObject<RandomDirection3dMobilityModel> ();
	  mob->AddObstacle(Box(0.0, 100.0, 0.0, 100.0, 0.0, 100.0));
	  mob->AddObstacle(Box(100, 200.0, 100, 200.0, 0.0, 200.0));
	}
      break;
    case Experiment::GAUSS_MARKOV:
      mobility.SetMobilityModel ("ns3::GaussMarkovMobilityModel",
				 "MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=20.0|Max=25.0]"),
				 "MeanPitch", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.14156]"),
				 "Alpha", DoubleValue (0.7),
				 "Bounds", BoxValue (Box (0.0, 300.0, 0.0, 300.0, 0.01, 200.0)));
      mobility.Install (sta);
      if (withObstacles)
	{
	  Ptr<GaussMarkovMobilityModel> mob = sta.Get(0)->GetObject<GaussMarkovMobilityModel> ();
	  mob->AddObstacle(Box(0.0, 100.0, 0.0, 100.0, 0.0, 100.0));
	  mob->AddObstacle(Box(100, 200.0, 100, 200.0, 0.0, 200.0));
	}
      break;
  }
  if (withObstacles)
    {
      Ptr<Building> b1 = CreateObject<Building> ();
      b1->SetBoundaries(Box(0.0, 100.0, 0.0, 100.0, 0.0, 100.0));
      b1->SetBuildingType(Building::Commercial);
      b1->SetExtWallsType(Building::StoneBlocks);
      Ptr<Building> b2 = CreateObject<Building> ();
      b2->SetBoundaries(Box(0.0, 100.0, 0.0, 100.0, 0.0, 100.0));
      b2->SetBuildingType(Building::Commercial);
      b2->SetExtWallsType(Building::StoneBlocks);
      BuildingsHelper::Install (sta);
      BuildingsHelper::MakeMobilityModelConsistent();
    }

  /*
   * PHY
   */
  string phyMode = "DsssRate1Mbps"; // PHY model
  WifiHelper wifi;
  wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue(-83));
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<YansWifiChannel> chan = wifiChannel.Create();
  if (withObstacles)
    {
      Ptr<OhBuildingsPropagationLossModel> prop = CreateObject<OhBuildingsPropagationLossModel> ();
      chan->SetPropagationLossModel(prop);
    }
  else
    {
      Ptr<OkumuraHataPropagationLossModel> prop = CreateObject<OkumuraHataPropagationLossModel> ();
      chan->SetPropagationLossModel(prop);
    }
  wifiPhy.SetChannel(chan);
  wifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default();
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode), "ControlMode", StringValue(phyMode));
  wifiMac.SetType("ns3::AdhocWifiMac");
  devices = wifi.Install(wifiPhy, wifiMac, sta);

  /*
   * Routing helper
   */
  Ipv4ListRoutingHelper list;
  OlsrHelper routing;
  list.Add (routing, 10);
  InternetStackHelper internet;
  internet.SetRoutingHelper (list);
  internet.Install (sta);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (256));
  //  Config::Connect("/NodeList/*/$ns3::MobilityModel/CourseChange",MakeCallback(&CourseChange));
  //  Config::Connect("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx",MakeCallback(&ReceivePacket));
  Config::Connect("/NodeList/*/$ns3::olsr::RoutingProtocol/Rx",MakeCallback(&Experiment::RxOLSR, this));
  Config::Connect("/NodeList/*/$ns3::olsr::RoutingProtocol/Tx",MakeCallback(&Experiment::TxOLSR, this));

  Simulator::Stop (Seconds (simtime));
  Simulator::Run ();
  Simulator::Destroy ();
  double result = 100*(((double)m_rxCount / (double)nodes) / (double)m_txCount);
  return result;
}

int main (int argc, char *argv[])
{
  double simtime = 200.0;

  uint32_t minNodes=1;
  uint32_t maxNodes=50;
  uint32_t speed = 25;
  /////////////////////////////////////////////////////////////////////////////////////////
  //
  //                           CPDR vs Nodes  (with obstacles)
  //
  /////////////////////////////////////////////////////////////////////////////////////////
  std::ofstream nodesFileO ("mob-results/obs-MOB-RESULT-NODES.plt");
  Gnuplot nodesPlotO = Gnuplot ("obs-MOB-RESULT-NODES.eps");
  Gnuplot2dDataset nodesDatasetO;
  nodesPlotO.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  nodesPlotO.SetLegend ("CPDR", "Number of nodes");
  //walk time
  nodesDatasetO = Gnuplot2dDataset("Random Walk Time");
  for (uint32_t avgnodes = minNodes; avgnodes <= maxNodes; avgnodes++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::WALK_TIME, avgnodes, speed, true);
      nodesDatasetO.Add(avgnodes, CPDR);
    }
  nodesPlotO.AddDataset(nodesDatasetO);
  nodesDatasetO = Gnuplot2dDataset("Random Walk Distance");
  for (uint32_t avgnodes = minNodes; avgnodes <= maxNodes; avgnodes++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::WALK_DISTANCE, avgnodes, speed, true);
      nodesDatasetO.Add(avgnodes, CPDR);
    }
  nodesPlotO.AddDataset(nodesDatasetO);
  nodesDatasetO = Gnuplot2dDataset("Random Direction");
  for (uint32_t avgnodes = minNodes; avgnodes <= maxNodes; avgnodes++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::DIRECTION, avgnodes, speed, true);
      nodesDatasetO.Add(avgnodes, CPDR);
    }
  nodesPlotO.AddDataset(nodesDatasetO);
  nodesDatasetO = Gnuplot2dDataset("Gauss-Markov");
  for (uint32_t avgnodes = minNodes; avgnodes <= maxNodes; avgnodes++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::GAUSS_MARKOV, avgnodes, speed, true);
      nodesDatasetO.Add(avgnodes, CPDR);
    }
  nodesPlotO.AddDataset(nodesDatasetO);
  nodesPlotO.GenerateOutput (nodesFileO);
  nodesFileO.close ();

  /////////////////////////////////////////////////////////////////////////////////////////
  //
  //                         CPDR vs Nodes  (without obstacles)
  //
  /////////////////////////////////////////////////////////////////////////////////////////
  std::ofstream nodesFileW ("mob-results/wobs-MOB-RESULT-NODES.plt");
  Gnuplot nodesPlotW = Gnuplot ("wobs-MOB-RESULT-NODES.eps");
  Gnuplot2dDataset nodesDatasetW;
  nodesPlotW.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  nodesPlotW.SetLegend ("CPDR", "Number of nodes");
  //walk time
  nodesDatasetW = Gnuplot2dDataset("Random Walk Time");
  for (uint32_t avgnodes = 1; avgnodes <= maxNodes; avgnodes++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::WALK_TIME, avgnodes, speed, false);
      nodesDatasetW.Add(avgnodes, CPDR);
    }
  nodesPlotW.AddDataset(nodesDatasetW);
  nodesDatasetW = Gnuplot2dDataset("Random Walk Distance");
  for (uint32_t avgnodes = 1; avgnodes <= maxNodes; avgnodes++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::WALK_DISTANCE, avgnodes, speed, false);
      nodesDatasetW.Add(avgnodes, CPDR);
    }
  nodesPlotW.AddDataset(nodesDatasetW);
  nodesDatasetW = Gnuplot2dDataset("Random Direction");
  for (uint32_t avgnodes = 1; avgnodes <= maxNodes; avgnodes++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::DIRECTION, avgnodes, speed, false);
      nodesDatasetW.Add(avgnodes, CPDR);
    }
  nodesPlotW.AddDataset(nodesDatasetW);
  nodesDatasetW = Gnuplot2dDataset("Gauss-Markov");
  for (uint32_t avgnodes = 1; avgnodes <= maxNodes; avgnodes++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::GAUSS_MARKOV, avgnodes, speed, false);
      nodesDatasetW.Add(avgnodes, CPDR);
    }
  nodesPlotW.AddDataset(nodesDatasetW);
  nodesPlotW.GenerateOutput (nodesFileW);
  nodesFileW.close ();











  uint32_t minSpeed=0;
  uint32_t maxSpeed=30;
  uint32_t nodes = 20;
  /////////////////////////////////////////////////////////////////////////////////////////
  //
  //                           CPDR vs Speed  (with obstacles)
  //
  /////////////////////////////////////////////////////////////////////////////////////////
  std::ofstream speedFileO ("mob-results/obs-MOB-RESULT-SPEED.plt");
  Gnuplot speedPlotO = Gnuplot ("obs-MOB-RESULT-SPEED.eps");
  Gnuplot2dDataset speedDatasetO;
  speedPlotO.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  speedPlotO.SetLegend ("CPDR", "Average speed of nodes");
  //walk time
  speedDatasetO = Gnuplot2dDataset("Random Walk Time");
  for (uint32_t avgspeed = minSpeed; avgspeed <= maxSpeed; avgspeed++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::WALK_TIME, nodes, avgspeed, true);
      speedDatasetO.Add(avgspeed, CPDR);
    }
  speedPlotO.AddDataset(speedDatasetO);
  speedDatasetO = Gnuplot2dDataset("Random Walk Distance");
  for (uint32_t avgspeed = minSpeed; avgspeed <= maxSpeed; avgspeed++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::WALK_DISTANCE, nodes, avgspeed, true);
      speedDatasetO.Add(avgspeed, CPDR);
    }
  speedPlotO.AddDataset(speedDatasetO);
  speedDatasetO = Gnuplot2dDataset("Random Direction");
  for (uint32_t avgspeed = minSpeed; avgspeed <= maxSpeed; avgspeed++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::DIRECTION, nodes, avgspeed, true);
      speedDatasetO.Add(avgspeed, CPDR);
    }
  speedPlotO.AddDataset(speedDatasetO);
  speedDatasetO = Gnuplot2dDataset("Gauss-Markov");
  for (uint32_t avgspeed = minSpeed; avgspeed <= maxSpeed; avgspeed++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::GAUSS_MARKOV, nodes, avgspeed, true);
      speedDatasetO.Add(avgspeed, CPDR);
    }
  speedPlotO.AddDataset(speedDatasetO);
  speedPlotO.GenerateOutput (speedFileO);
  speedFileO.close ();

  /////////////////////////////////////////////////////////////////////////////////////////
  //
  //                         CPDR vs Speed  (without obstacles)
  //
  /////////////////////////////////////////////////////////////////////////////////////////
  std::ofstream speedFileW ("mob-results/wobs-MOB-RESULT-SPEED.plt");
  Gnuplot speedPlotW = Gnuplot ("wobs-MOB-RESULT-SPEED.eps");
  Gnuplot2dDataset speedDatasetW;
  speedPlotW.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  speedPlotW.SetLegend ("CPDR", "Average speed of nodes");
  //walk time
  speedDatasetW = Gnuplot2dDataset("Random Walk Time");
  for (uint32_t avgspeed = minSpeed; avgspeed <= maxSpeed; avgspeed++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::WALK_TIME, nodes, avgspeed, false);
      speedDatasetW.Add(avgspeed, CPDR);
    }
  speedPlotW.AddDataset(speedDatasetW);
  speedDatasetW = Gnuplot2dDataset("Random Walk Distance");
  for (uint32_t avgspeed = minSpeed; avgspeed <= maxSpeed; avgspeed++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::WALK_DISTANCE, nodes, avgspeed, false);
      speedDatasetW.Add(avgspeed, CPDR);
    }
  speedPlotW.AddDataset(speedDatasetW);
  speedDatasetW = Gnuplot2dDataset("Random Direction");
  for (uint32_t avgspeed = minSpeed; avgspeed <= maxSpeed; avgspeed++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::DIRECTION, nodes, avgspeed, false);
      speedDatasetW.Add(avgspeed, CPDR);
    }
  speedPlotW.AddDataset(speedDatasetW);
  speedDatasetW = Gnuplot2dDataset("Gauss-Markov");
  for (uint32_t avgspeed = minSpeed; avgspeed <= maxSpeed; avgspeed++)
    {
      Experiment experiment;
      experiment = Experiment ();
      double CPDR = experiment.Run(simtime, Experiment::GAUSS_MARKOV, nodes, avgspeed, false);
      speedDatasetW.Add(avgspeed, CPDR);
    }
  speedPlotW.AddDataset(speedDatasetW);
  speedPlotW.GenerateOutput (speedFileW);
  speedFileW.close ();







  return 0;
}
