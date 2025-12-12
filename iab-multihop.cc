#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-static-routing-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("IabMultiHopSim");

int
main (int argc, char *argv[])
{
  uint32_t hops = 1;  // number of wireless hops: 1, 2, or 3

  CommandLine cmd;
  cmd.AddValue ("hops", "Number of wireless hops (1, 2, or 3)", hops);
  cmd.Parse (argc, argv);

  if (hops < 1 || hops > 3)
    {
      std::cout << "Invalid hops=" << hops << ", forcing to 1\n";
      hops = 1;
    }

  std::cout << "Running IAB-like multi-hop simulation with "
            << hops << " hop(s)" << std::endl;

  // Total nodes: gNB + UE + (hops - 1) relays
  // But using a chain of (hops + 1) nodes:
  // Node 0: gNB
  // Node 1..hops-1: relays (if any)
  // Node hops: UE
  uint32_t totalNodes = hops + 1;
  NodeContainer nodes;
  nodes.Create (totalNodes);

  // Wi-Fi ad-hoc to emulate wireless IAB links
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211ac);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy;
  phy.SetChannel (channel.Create ());
  phy.Set ("TxPowerStart", DoubleValue (20.0));
  phy.Set ("TxPowerEnd",   DoubleValue (20.0));
  phy.Set ("RxSensitivity", DoubleValue (-95.0));

  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");

  NetDeviceContainer devices = wifi.Install (phy, mac, nodes);

  // Mobility: place nodes in a line with 50m spacing
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator> ();

  for (uint32_t i = 0; i < totalNodes; ++i)
    {
      posAlloc->Add (Vector (50.0 * i, 0.0, 0.0));
    }

  mobility.SetPositionAllocator (posAlloc);
  mobility.Install (nodes);

  // Internet stack and IP addressing
  InternetStackHelper stack;
  stack.Install (nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.1.0.0", "255.255.255.0");
  Ipv4InterfaceContainer ifaces = ipv4.Assign (devices);

  Ipv4Address srcAddr = ifaces.GetAddress (0);          // gNB
  Ipv4Address dstAddr = ifaces.GetAddress (hops);       // UE

  // Static routing: force traffic along the chain
  Ipv4StaticRoutingHelper staticHelper;

  for (uint32_t i = 0; i < hops; ++i)
    {
      Ptr<Ipv4> ipv4Node = nodes.Get (i)->GetObject<Ipv4> ();
      Ptr<Ipv4StaticRouting> routing = staticHelper.GetStaticRouting (ipv4Node);

      // Next hop is node (i+1)
      Ipv4Address nextHop = ifaces.GetAddress (i + 1);
      // Interface index 1 is the Wifi (0 is loopback)
      routing->AddHostRouteTo (dstAddr, nextHop, 1);
    }

  // UDP traffic: gNB (node 0) -> UE (node hops)
  uint16_t port = 5000;
  UdpServerHelper server (port);
  ApplicationContainer serverApps = server.Install (nodes.Get (hops));
  serverApps.Start (Seconds (0.5));
  serverApps.Stop (Seconds (12.0));

  UdpClientHelper client (dstAddr, port);
  client.SetAttribute ("MaxPackets", UintegerValue (2000000));
  client.SetAttribute ("Interval", TimeValue (MicroSeconds (200))); // 5 kpps
  client.SetAttribute ("PacketSize", UintegerValue (1200));

  ApplicationContainer clientApps = client.Install (nodes.Get (0));
  clientApps.Start (Seconds (1.0));
  clientApps.Stop (Seconds (11.0));

  // FlowMonitor to measure throughput and delay
  FlowMonitorHelper flowmonHelper;
  Ptr<FlowMonitor> flowmon = flowmonHelper.InstallAll ();

  Simulator::Stop (Seconds (13.0));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier> (flowmonHelper.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = flowmon->GetFlowStats ();

  for (const auto &flow : stats)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (flow.first);

      if (t.sourceAddress == srcAddr && t.destinationAddress == dstAddr)
        {
          uint32_t txPkts = flow.second.txPackets;
          uint32_t rxPkts = flow.second.rxPackets;
          uint64_t rxBytes = flow.second.rxBytes;

          double throughputMbps = 0.0;
          double avgDelayMs = 0.0;
          double plr = 0.0;

          if (rxPkts > 0 &&
              flow.second.timeLastRxPacket > flow.second.timeFirstTxPacket)
            {
              double duration =
                flow.second.timeLastRxPacket.GetSeconds () -
                flow.second.timeFirstTxPacket.GetSeconds ();
              throughputMbps = (rxBytes * 8.0) / (duration * 1e6);
              avgDelayMs =
                (flow.second.delaySum.GetSeconds () / rxPkts) * 1000.0;
            }

          if (txPkts > 0)
            {
              plr = (txPkts - rxPkts) * 100.0 / txPkts;
            }

          std::cout << "=== IAB-like Multi-hop Results ===" << std::endl;
          std::cout << "Hops:        " << hops << std::endl;
          std::cout << "Throughput:  " << throughputMbps << " Mbps" << std::endl;
          std::cout << "Avg delay:   " << avgDelayMs << " ms" << std::endl;
          std::cout << "Packet loss: " << plr << " %" << std::endl;

// save file
std::cout << "CSV,"
          << hops << ","
          << throughputMbps << ","
          << avgDelayMs << ","
          << plr
          << std::endl;

}
  Simulator::Destroy ();
  return 0;

}
}
