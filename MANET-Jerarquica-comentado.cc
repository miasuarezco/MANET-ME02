/**
 * @file
 * @brief Simulates a hierarchical MANET with two levels of mobility.
 *
 * This script models a MANET with a two-level hierarchy.
 * Level 2: A single "Super-Leader" moves randomly across the area.
 * Level 1: Two "Cluster-Leaders" follow the Super-Leader in a fixed formation.
 * Level 0: Follower nodes within each cluster follow their respective Cluster-Leader.
 *
 * This creates a system where entire clusters move together (cluster-level mobility)
 * and individual nodes move within their cluster (node-level mobility).
 * Inter-cluster routing is handled by a dedicated backbone network for leaders using OLSR+HNA.
 */

//================================================================================
// 1. INCLUDES & NAMESPACE
//================================================================================
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/log.h"
#include "ns3/olsr-helper.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/netanim-module.h"
#include "ns3/ipv4.h"

#include <fstream>
#include <iomanip>

using namespace ns3;
using namespace ns3::olsr;

//================================================================================
// 2. GLOBAL VARIABLES & LOGGING
//================================================================================
NS_LOG_COMPONENT_DEFINE("HierarchicalMobilityMANET");

//================================================================================
// 3. FUNCTION PROTOTYPES
//================================================================================
void RunSimulation(uint32_t nodesPerCluster, double simulationTime, double areaSize, double followerSpeed, double noiseFactor);
void UpdateHierarchicalMobility(Ptr<Node> superLeader, Ptr<Node> clusterLeaderA, Ptr<Node> clusterLeaderB, NodeContainer followersA, NodeContainer followersB, double followerSpeed, double noiseFactor);

//================================================================================
// 4. MAIN FUNCTION
//================================================================================
int main(int argc, char *argv[]) {
    // --- Simulation Parameters ---
    uint32_t nodesPerCluster = 5;
    double simulationTime = 160.0; // seconds
    double areaSize = 200.0;       // 200x200 meters
    double followerSpeed = 1.5;    // m/s
    double noiseFactor = 1.0;      // randomness in follower movement

    // --- Command Line Parser for customization ---
    CommandLine cmd;
    cmd.AddValue("nodesPerCluster", "Number of follower nodes per cluster", nodesPerCluster);
    cmd.AddValue("simTime", "Total simulation time in seconds", simulationTime);
    cmd.AddValue("areaSize", "Side length of the simulation area in meters", areaSize);
    cmd.AddValue("followerSpeed", "Speed of follower nodes in m/s", followerSpeed);
    cmd.AddValue("noiseFactor", "Noise factor for follower movement", noiseFactor);
    cmd.Parse(argc, argv);

    // --- Run Simulation ---
    RunSimulation(nodesPerCluster, simulationTime, areaSize, followerSpeed, noiseFactor);

    return 0;
}

//================================================================================
// 5. CORE SIMULATION LOGIC
//================================================================================

/**
 * @brief Configures and runs the hierarchical MANET simulation.
 */
void RunSimulation(uint32_t nodesPerCluster, double simulationTime, double areaSize, double followerSpeed, double noiseFactor) {
    // --- Node Creation ---
    // Level 2
    NodeContainer superLeaderContainer;
    superLeaderContainer.Create(1);
    Ptr<Node> superLeader = superLeaderContainer.Get(0);

    // Level 1
    NodeContainer clusterLeadersContainer;
    clusterLeadersContainer.Create(2);
    Ptr<Node> clusterLeaderA = clusterLeadersContainer.Get(0);
    Ptr<Node> clusterLeaderB = clusterLeadersContainer.Get(1);

    // Level 0
    NodeContainer followersA, followersB;
    followersA.Create(nodesPerCluster - 1); // -1 because the leader is also part of the cluster
    followersB.Create(nodesPerCluster - 1);

    // Full cluster containers for convenience
    NodeContainer clusterA_nodes = followersA;
    clusterA_nodes.Add(clusterLeaderA);
    NodeContainer clusterB_nodes = followersB;
    clusterB_nodes.Add(clusterLeaderB);
    

    Ptr<RandomRectanglePositionAllocator> alloc = CreateObject<RandomRectanglePositionAllocator>();

    // Super-Leader mobilitySuperLeader model

    Ptr<WaypointMobilityModel> mobilitySuperLeader = CreateObject<WaypointMobilityModel>();
    superLeader->AggregateObject(mobilitySuperLeader);

    // Posición inicial
    mobilitySuperLeader->AddWaypoint(Waypoint(Seconds(0.0), Vector(50.0, 50.0, 0.0)));

    // Reubicación en tiempo aleatorio
    // Tiempo aleatorio entre la mitad y el final de la simulación

    // Nueva posición aleatoria dentro de un rango (ej. 200x200m)
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
    uv->SetAttribute("Min", DoubleValue(simulationTime / 2.0));
    uv->SetAttribute("Max", DoubleValue(simulationTime));
    double moveTime = uv->GetValue();

    Ptr<UniformRandomVariable> posX = CreateObject<UniformRandomVariable>();
    posX->SetAttribute("Min", DoubleValue(0.0));
    posX->SetAttribute("Max", DoubleValue(200.0));
    double x = posX->GetValue();

    Ptr<UniformRandomVariable> posY = CreateObject<UniformRandomVariable>();
    posY->SetAttribute("Min", DoubleValue(0.0));
    posY->SetAttribute("Max", DoubleValue(200.0));
    double y = posY->GetValue();
    Vector newPosition(x, y, 0.0);

    // Añadir waypoint
    mobilitySuperLeader->AddWaypoint(Waypoint(Seconds(moveTime), newPosition));
    

    // lideres mobilidad
    
    Ptr<PositionAllocator> positionAlloc = CreateObject<RandomRectanglePositionAllocator>();
    positionAlloc->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=200.0]"));
    positionAlloc->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=200.0]"));

    MobilityHelper mobilityLeaders;
    mobilityLeaders.SetMobilityModel("ns3::RandomWaypointMobilityModel",
        "Speed", StringValue("ns3::UniformRandomVariable[Min=0.5|Max=1.5]"), // movimiento lento
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),  // pausas realistas
        "PositionAllocator", PointerValue(positionAlloc));
    

    mobilityLeaders.SetPositionAllocator(positionAlloc);
    mobilityLeaders.Install(clusterLeadersContainer);
    
    MobilityHelper mobility;
    // seguidores mobilidad
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(followersA);
    mobility.Install(followersB);
    
    std::cout << "Hola desde el simulador" << std::endl;
    
    // --- Channel, PHY, and MAC Setup ---
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211n);
    
    // --- Network Stack and Protocol Setup ---
    InternetStackHelper internet;
    OlsrHelper olsr;
    internet.SetRoutingHelper(olsr);
    internet.Install(superLeaderContainer);
    internet.Install(clusterLeadersContainer);
    internet.Install(followersA);
    internet.Install(followersB);

    // --- IP Addressing (3 Subnets) ---
    Ipv4AddressHelper address;
    
    // Subnet 1: Backbone network for leaders
    address.SetBase("192.168.1.0", "255.255.255.0");
    NetDeviceContainer backboneDevices = wifi.Install(wifiPhy, wifiMac, NodeContainer(superLeader, clusterLeaderA, clusterLeaderB));
    Ipv4InterfaceContainer backboneInterfaces = address.Assign(backboneDevices);

    // Subnet 2: Cluster A network
    address.SetBase("10.1.1.0", "255.255.255.0");
    NetDeviceContainer clusterADevices = wifi.Install(wifiPhy, wifiMac, clusterA_nodes);
    Ipv4InterfaceContainer clusterAInterfaces = address.Assign(clusterADevices);
    std::cout<< "Cluster A IP in BASE: 10.1.1.0" << std::endl;

    // Subnet 3: Cluster B network
    address.SetBase("10.1.2.0", "255.255.255.0");
    NetDeviceContainer clusterBDevices = wifi.Install(wifiPhy, wifiMac, clusterB_nodes);
    Ipv4InterfaceContainer clusterBInterfaces = address.Assign(clusterBDevices);
    
    
    // --- Enable IP Forwarding and Configure HNA for Inter-Cluster Routing ---
    // Leaders need IP forwarding to route packets between their interfaces.
    superLeader->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    clusterLeaderA->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    clusterLeaderB->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));

    // Cluster Leader A advertises its local network (10.1.1.0/24) to the backbone.
    Ptr<olsr::RoutingProtocol> olsrA = clusterLeaderA->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<olsr::RoutingProtocol>();
    olsrA->AddHostNetworkAssociation(Ipv4Address("10.1.1.0"), Ipv4Mask("255.255.255.0"));
    
    // Cluster Leader B advertises its local network (10.1.2.0/24) to the backbone.
    Ptr<olsr::RoutingProtocol> olsrB = clusterLeaderB->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<olsr::RoutingProtocol>();
    olsrB->AddHostNetworkAssociation(Ipv4Address("10.1.2.0"), Ipv4Mask("255.255.255.0"));

    // --- Application Setup (Telemetria desde seguidores a lideres)
    
    uint16_t telemetryPort = 9;

    // --- Sink apps en líderes ---
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), telemetryPort));

    ApplicationContainer sinkApps;
    sinkApps.Add(sink.Install(clusterLeaderA));
    sinkApps.Add(sink.Install(clusterLeaderB));
    sinkApps.Start(Seconds(1.0));
    sinkApps.Stop(Seconds(simulationTime));

    // --- Telemetría desde seguidores A hacia clusterLeaderA ---
    Ipv4Address leaderAIp = clusterAInterfaces.GetAddress(0); 
    std::cout<< "Leader A IP: " << leaderAIp << std::endl;
    for (uint32_t i = 0; i < followersA.GetN(); ++i) {
        OnOffHelper source("ns3::UdpSocketFactory", InetSocketAddress(leaderAIp, telemetryPort));
        source.SetConstantRate(DataRate("256kbps"));
        source.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer app = source.Install(followersA.Get(i));
        app.Start(Seconds(2.0));
        app.Stop(Seconds(simulationTime - 2.0));
    }

    // --- Telemetría desde seguidores B hacia clusterLeaderB ---
    Ipv4Address leaderBIp = clusterBInterfaces.GetAddress(0); // Última IP = líder
    for (uint32_t i = 0; i < followersB.GetN(); ++i) {
        OnOffHelper source("ns3::UdpSocketFactory", InetSocketAddress(leaderBIp, telemetryPort));
        source.SetConstantRate(DataRate("256kbps"));
        source.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer app = source.Install(followersB.Get(i));
        app.Start(Seconds(2.0));
        app.Stop(Seconds(simulationTime - 2.0));
    }



    AnimationInterface anim("HierarchicalMobility.xml");
    anim.SetConstantPosition(superLeader, 10, 10); // Initial placeholder positions
    anim.SetConstantPosition(clusterLeaderA, 20, 20);
    anim.SetConstantPosition(clusterLeaderB, 30, 30);
    
    // --- Schedule Mobility Updates ---
    // This is the core function that drives the custom hierarchical mobility.
    double updateInterval = 0.1; // seconds
    Simulator::Schedule(Seconds(0.1), &UpdateHierarchicalMobility, superLeader, clusterLeaderA, clusterLeaderB, followersA, followersB, followerSpeed, noiseFactor);
    
    std::cout << "Fin de configuracion de simulacion, empezando simulacion" << std::endl;
    // --- Post-Simulation Analysis ---
    // --- Monitoring & Visualization ---
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // --- Run Simulation ---
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();

    std::cout << "Fin simulacion, datos" << std::endl;
    
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    for (auto it = stats.begin(); it != stats.end(); ++it) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(it->first);
        NS_LOG_UNCOND("Flow " << it->first << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")");
        NS_LOG_UNCOND("  Tx Packets: " << it->second.txPackets);
        NS_LOG_UNCOND("  Rx Packets: " << it->second.rxPackets);
        NS_LOG_UNCOND("  PDR: " << (double)it->second.rxPackets / it->second.txPackets * 100 << "%");
        if (it->second.rxPackets > 0) {
            NS_LOG_UNCOND("  Avg Latency: " << it->second.delaySum.GetMilliSeconds() / it->second.rxPackets << " ms");
        }
    }

    // --- Cleanup ---
    Simulator::Destroy();
    
    
}

ns3::Vector Normalize(const ns3::Vector& v) {
    double mag = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return (mag != 0) ? ns3::Vector(v.x / mag, v.y / mag, v.z / mag) : ns3::Vector(0, 0, 0);
}
//================================================================================
// 6. HIERARCHICAL MOBILITY LOGIC
//================================================================================

/**
 * @brief Periodically updates the positions of all nodes according to the hierarchical model.
 *
 * 1.  It gets the Super-Leader's current position.
 * 2.  It sets the Cluster-Leaders' positions to maintain a fixed formation around the Super-Leader.
 * 3.  It updates the velocity of follower nodes to move towards their respective Cluster-Leader.
 *
 * @param superLeader Pointer to the top-level leader node.
 * @param clusterLeaderA Pointer to the leader of cluster A.
 * @param clusterLeaderB Pointer to the leader of cluster B.
 * @param followersA NodeContainer for followers in cluster A.
 * @param followersB NodeContainer for followers in cluster B.
 * @param followerSpeed The speed of follower nodes.
 * @param noiseFactor The randomness factor in follower movement.
 */
void UpdateHierarchicalMobility(Ptr<Node> superLeader, Ptr<Node> clusterLeaderA, Ptr<Node> clusterLeaderB, NodeContainer followersA, NodeContainer followersB, double followerSpeed, double noiseFactor) {
    // --- Get Current Position of Super-Leader (Level 2) ---
    Vector superLeaderPos = superLeader->GetObject<MobilityModel>()->GetPosition();

    // --- Update Cluster-Leader Positions (Level 1 Mobility) ---
    // They maintain a fixed offset from the super-leader, creating a formation.
    Vector offsetA(-50, -50, 0); // Cluster A is bottom-left of the super-leader
    Vector offsetB(50, 50, 0);   // Cluster B is top-right of the super-leader
    
    Ptr<MobilityModel> mobilityA = clusterLeaderA->GetObject<MobilityModel>();
    mobilityA->SetPosition(superLeaderPos + offsetA);

    Ptr<MobilityModel> mobilityB = clusterLeaderB->GetObject<MobilityModel>();
    mobilityB->SetPosition(superLeaderPos + offsetB);

    // --- Update Follower Velocities (Level 0 Mobility) ---
    Ptr<UniformRandomVariable> noise = CreateObject<UniformRandomVariable>();

    // Update followers in Cluster A
    Vector leaderAPos = mobilityA->GetPosition();
    for (uint32_t i = 0; i < followersA.GetN(); ++i) {
        Ptr<MobilityModel> followerMobility = followersA.Get(i)->GetObject<MobilityModel>();
        Vector followerPos = followerMobility->GetPosition();
        Vector direction = leaderAPos - followerPos;
        Vector velocity = Normalize(direction) * followerSpeed + Vector(noise->GetValue(-noiseFactor, noiseFactor), noise->GetValue(-noiseFactor, noiseFactor), 0);
        followerMobility->SetPosition(followerPos + velocity * 0.1); // Simple Euler integration
    }

    // Update followers in Cluster B
    Vector leaderBPos = mobilityB->GetPosition();
    for (uint32_t i = 0; i < followersB.GetN(); ++i) {
        Ptr<MobilityModel> followerMobility = followersB.Get(i)->GetObject<MobilityModel>();
        Vector followerPos = followerMobility->GetPosition();
        Vector direction = leaderBPos - followerPos;
        Vector velocity = Normalize(direction) * followerSpeed + Vector(noise->GetValue(-noiseFactor, noiseFactor), noise->GetValue(-noiseFactor, noiseFactor), 0);
        followerMobility->SetPosition(followerPos + velocity * 0.1); // Simple Euler integration
    }
    
    // Re-schedule this function to maintain continuous movement.
    Simulator::Schedule(Seconds(0.1), &UpdateHierarchicalMobility, superLeader, clusterLeaderA, clusterLeaderB, followersA, followersB, followerSpeed, noiseFactor);
}