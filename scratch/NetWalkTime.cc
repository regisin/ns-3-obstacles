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
#include <fstream>

using namespace ns3;
using namespace std;

uint32_t rxCount = 0;
uint32_t txCount = 0;
uint32_t numNodes = 2;
uint32_t avgSpeed = 25;

void
PrintResults()
{
  std::cout<<"###########\tRandom Walk Time\t###########" << std::endl;
  std::cout<<"Nodes: " << numNodes << std::endl;
  std::cout<<"Speed: " << avgSpeed << std::endl;
  std::cout<<"Total control packets received: " << rxCount << std::endl;
  std::cout<<"Total control packets sent: " << txCount << std::endl;
  std::cout<<"Control packets delivery ratio: " << 100*(((double)rxCount / (double)numNodes) / (double)txCount) << " %" << std::endl;
}
void
RxOLSR (std::string str, const olsr::PacketHeader &header, const olsr::MessageList &messages)
{
  rxCount += 1;
}
void
TxOLSR (std::string str, const olsr::PacketHeader &header, const olsr::MessageList &messages)
{
  txCount += 1;
}

int main (int argc, char *argv[])
{
  double simTimeSec=200.0;
  bool withObstacles = true;

  CommandLine cmd;
  cmd.AddValue ("time",  "Simulation time (seconds)", simTimeSec);
  cmd.AddValue ("nodes",  "Number of traffic generating nodes", numNodes);
  cmd.AddValue ("speed",  "Speed of the nodes (m/s)", avgSpeed);
  cmd.AddValue ("obstacles",  "With obstacles (true or false)?", withObstacles);
  cmd.Parse (argc,argv);

  std::stringstream avgSpeedStr;
  avgSpeedStr << "ns3::ConstantRandomVariable[Constant=" << avgSpeed << "]";

  NodeContainer sta;
  NetDeviceContainer devices;
  sta.Create (numNodes);
  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
				 "X", StringValue ("ns3::UniformRandomVariable[Min=200.0|Max=300.0]"),
				 "Y", StringValue ("ns3::UniformRandomVariable[Min=200.0|Max=300.0]"),
				 "Z", StringValue ("ns3::UniformRandomVariable[Min=0.01|Max=200.0]"));

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
  Config::Connect("/NodeList/*/$ns3::olsr::RoutingProtocol/Rx",MakeCallback(&RxOLSR));
  Config::Connect("/NodeList/*/$ns3::olsr::RoutingProtocol/Tx",MakeCallback(&TxOLSR));

  Simulator::Stop (Seconds (simTimeSec));
  Simulator::Run ();
  Simulator::Destroy ();

  PrintResults();
  return 0;
}
