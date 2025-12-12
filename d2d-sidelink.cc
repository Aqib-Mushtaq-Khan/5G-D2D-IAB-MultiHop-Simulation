#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-static-routing-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("D2dSidelinkSim");

int
main (int argc, char *argv[])
{
std::string mode = "direct";  // "direct" or "relay"
double intervalUs = 100.0;    // packet interval in microseconds (baseline = 100us)

CommandLine cmd;
cmd.AddValue ("mode", "Simulation mode: direct or relay", mode);
cmd.AddValue ("intervalUs", "UDP packet interval in microseconds", intervalUs);
cmd.Parse (argc, argv);



  std::cout << "Running D2D Sidelink Simulation, mode = " << mode << std::endl;

  // 1. Nodes: UE1, Relay, UE2
  NodeContainer nodes;
  nodes.Create (3);

  // 2. WiFi ad-hoc as sidelink
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211ac);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  // Limit range so UE1-UE2 is too far in relay mode
  //channel.AddPropagationLoss ("ns3::RangePropagationLossModel",
    //                          "MaxRange", DoubleValue (120.0));

  YansWifiPhyHelper phy;
  phy.SetChannel (channel.Create ());
  phy.Set ("TxPowerStart", DoubleValue (20.0));
  phy.Set ("TxPowerEnd",   DoubleValue (20.0));
  phy.Set ("RxSensitivity", DoubleValue (-95.0));

  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install (phy, mac, nodes);

  // 3. Mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator> ();

  if (mode == "direct")
    {
      // UE1 ---- 50m ---- UE2 (relay in the middle, not needed)
      posAlloc->Add (Vector (0.0, 0.0, 0.0));    // UE1
      posAlloc->Add (Vector (25.0, 0.0, 0.0));   // Relay
      posAlloc->Add (Vector (50.0, 0.0, 0.0));   // UE2
    }
else // relay mode
  {
    // UE1 --- 40m --- Relay --- 40m --- UE2 (very strong links)
    posAlloc->Add (Vector (0.0, 0.0, 0.0));     // UE1
    posAlloc->Add (Vector (40.0, 0.0, 0.0));    // Relay
    posAlloc->Add (Vector (80.0, 0.0, 0.0));    // UE2
  }

  mobility.SetPositionAllocator (posAlloc);
  mobility.Install (nodes);

  // 4. Internet stack (no OLSR, we use static routes)
  InternetStackHelper stack;
  stack.Install (nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer ifaces = ipv4.Assign (devices);

  Ipv4Address ue1Addr   = ifaces.GetAddress (0);
  Ipv4Address relayAddr = ifaces.GetAddress (1);
  Ipv4Address ue2Addr   = ifaces.GetAddress (2);

  // 4b. Static routing: only needed for relay mode
  if (mode == "relay")
    {
      Ipv4StaticRoutingHelper staticHelper;

      Ptr<Ipv4StaticRouting> ue1Routing =
        staticHelper.GetStaticRouting (nodes.Get (0)->GetObject<Ipv4> ());
      Ptr<Ipv4StaticRouting> relayRouting =
        staticHelper.GetStaticRouting (nodes.Get (1)->GetObject<Ipv4> ());

      // UE1: send packets for UE2 via Relay
      ue1Routing->AddHostRouteTo (ue2Addr, relayAddr, 1);

      // Relay: forward packets for UE2 directly to UE2
      relayRouting->AddHostRouteTo (ue2Addr, ue2Addr, 1);
    }

  // 5. UDP traffic: UE1 → UE2
  uint16_t port = 5000;

  UdpServerHelper server (port);
  ApplicationContainer serverApps = server.Install (nodes.Get (2));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (13.0));

  UdpClientHelper client (ue2Addr, port);
  client.SetAttribute ("MaxPackets", UintegerValue (2000000));
client.SetAttribute ("Interval", TimeValue (MicroSeconds (intervalUs)));

  client.SetAttribute ("PacketSize", UintegerValue (1400));
  ApplicationContainer clientApps = client.Install (nodes.Get (0));
  clientApps.Start (Seconds (4.0));    // give time for routes to be ready
  clientApps.Stop  (Seconds (12.0));

  // 6. FlowMonitor
  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> flowmon = flowmonHelper.InstallAll ();

  Simulator::Stop (Seconds (14.0));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = flowmon->GetFlowStats ();

  for (const auto &flow : stats)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (flow.first);

      if (t.sourceAddress == ue1Addr &&
          t.destinationAddress == ue2Addr)
        {
          uint32_t txPkts = flow.second.txPackets;
          uint32_t rxPkts = flow.second.rxPackets;

          double throughputMbps = 0.0;
          double avgDelayMs = 0.0;
          double plr = 0.0;

          if (rxPkts > 0 &&
              flow.second.timeLastRxPacket > flow.second.timeFirstTxPacket)
            {
              double duration =
                flow.second.timeLastRxPacket.GetSeconds () -
                flow.second.timeFirstTxPacket.GetSeconds ();

              throughputMbps =
                (flow.second.rxBytes * 8.0) / (duration * 1e6);
              avgDelayMs =
                (flow.second.delaySum.GetSeconds () / rxPkts) * 1000.0;
            }

          if (txPkts > 0)
            {
              plr = (txPkts - rxPkts) * 100.0 / txPkts;
            }

          std::cout << "=== Results (" << mode << ") ===" << std::endl;
          std::cout << "Throughput:  " << throughputMbps << " Mbps\n";
          std::cout << "Avg delay:   " << avgDelayMs << " ms\n";
          std::cout << "Packet loss: " << plr << " %\n";


// Saved in SSV file:
std::cout << "CSV,"
          << mode << ","
          << intervalUs << ","
          << throughputMbps << ","
          << avgDelayMs << ","
          << plr
          << std::endl;
}
  Simulator::Destroy ();
  return 0;
}

}
