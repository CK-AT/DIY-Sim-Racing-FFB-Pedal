// Plan 17 — MSFS SimConnect bridge. Standalone EXE that connects to MSFS
// 2024 via SimConnect, samples helicopter SimVars, and sends a 152-byte
// UDP packet at sim-frame rate (~50 Hz) to the SimHub plugin on port 27016.
//
// Bridge ships raw SimVars only. All derivations (BladeAlph / VRS / Slap /
// Propwash / Torque from IAS + descent rate + weight) live in the SimHub
// plugin (GraphSignals.BuildMsfsInputs) so per-aircraft tuning happens
// through the existing graph-param mechanism, not by rebuilding C++.
//
// Packet contract: see SimHubPlugin/DiyFfbPlugin.cs MsfsUdpPacket +
// ParseMsfsPacket. Mirrors XPlanePlugin/DiyFfbDataProvider.cpp design.

#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <chrono>
#include <thread>

#include <windows.h>
#include <SimConnect.h>

namespace {

//-----------------------------------------------------------------------------
// Packet contract (must match SimHubPlugin/DiyFfbPlugin.cs MsfsUdpPacket).

#pragma pack(push, 1)
struct MsfsFfbPacket {
    uint32_t magic;               //  0  'MFFB' = 0x4D464642
    uint16_t version;             //  4  1
    uint16_t size_bytes;          //  6  152
    uint32_t sequence;            //  8
    // Native non-rotor SimVars
    float ias_kts;                // 12
    float tas_kts;                // 16
    float alpha_deg;              // 20
    float beta_deg;               // 24
    float p_rate_rad_s;           // 28
    float q_rate_rad_s;           // 32
    float r_rate_rad_s;           // 36
    float g_force;                // 40
    float vvi_world_fps;          // 44
    float velocity_body_x_fps;    // 48
    float velocity_body_y_fps;    // 52
    float velocity_body_z_fps;    // 56
    float pitch_rad;              // 60
    float bank_rad;               // 64
    float total_weight_lb;        // 68
    float ambient_density_slugs_ft3; // 72
    // Native rotor / heli SimVars
    float main_rotor_rpm;         // 76
    float tail_rotor_rpm;         // 80
    float eng_torque_pct;         // 84
    float collective_pos_pct;     // 88
    float tail_rotor_pedal_pct;   // 92
    float tail_rotor_blade_pitch_pct;        // 96
    float rotor_collective_blade_pitch_pct;  // 100
    float rotor_cyclic_blade_pitch_pct;      // 104
    float rotor_cyclic_blade_max_pitch_pos_rad; // 108
    float disk_pitch_angle_rad;   // 112
    float disk_bank_angle_rad;    // 116
    float disk_coning_pct;        // 120
    float rotor_lateral_trim_pct; // 124
    float rotor_longitudinal_trim_pct; // 128
    float rotor_rotation_angle_rad;    // 132
    // Trim
    float elev_trim_pct;          // 136
    float ail_trim_pct;           // 140
    float rud_trim_pct;           // 144
    uint8_t on_ground;            // 148
    uint8_t reserved[3];          // 149-151
};
#pragma pack(pop)
static_assert(sizeof(MsfsFfbPacket) == 152, "MsfsFfbPacket must be 152 bytes");

constexpr uint32_t kPacketMagic = 0x4D464642; // 'MFFB' (little-endian)
constexpr uint16_t kPacketVersion = 1;

//-----------------------------------------------------------------------------
// SimConnect data definitions. Order MUST match the SimVarSample struct fields.

enum DataDefineId {
    DEF_FFB_SAMPLE = 1,
};

enum DataRequestId {
    REQ_FFB_SAMPLE = 1,
};

// Layout matched 1:1 with SimConnect_AddToDataDefinition order. All fields
// are FLOAT64 because that's the SimConnect default and avoids surprises.
struct SimVarSample {
    double airspeed_indicated_kts;        // AIRSPEED INDICATED, Knots
    double airspeed_true_kts;             // AIRSPEED TRUE, Knots
    double incidence_alpha_deg;           // INCIDENCE ALPHA, Degrees
    double incidence_beta_deg;            // INCIDENCE BETA, Degrees
    double rotation_velocity_body_x_rad_s; // ROTATION VELOCITY BODY X, Radians per second
    double rotation_velocity_body_y_rad_s;
    double rotation_velocity_body_z_rad_s;
    double g_force;                       // G FORCE, GForce
    double velocity_world_y_fps;          // VELOCITY WORLD Y, Feet per second
    double velocity_body_x_fps;           // VELOCITY BODY X, Feet per second
    double velocity_body_y_fps;
    double velocity_body_z_fps;
    double plane_pitch_rad;               // PLANE PITCH DEGREES, Radians (despite the name)
    double plane_bank_rad;                // PLANE BANK DEGREES, Radians
    double total_weight_lb;               // TOTAL WEIGHT, Pounds
    double ambient_density_slugs_ft3;     // AMBIENT DENSITY, Slugs per cubic feet
    double main_rotor_rpm;                // ROTOR RPM:1, Rpm
    double tail_rotor_rpm;                // ROTOR RPM:2, Rpm
    double eng_torque_percent;            // ENG TORQUE PERCENT:1, Percent
    double collective_position_pct;       // COLLECTIVE POSITION, Percent
    double tail_rotor_pedal_pct;          // TAIL ROTOR PEDAL POSITION, Percent
    double tail_rotor_blade_pitch_pct;    // TAIL ROTOR BLADE PITCH PCT, Percent
    double rotor_collective_blade_pitch_pct; // ROTOR COLLECTIVE BLADE PITCH PCT, Percent
    double rotor_cyclic_blade_pitch_pct;     // ROTOR CYCLIC BLADE PITCH PCT, Percent
    double rotor_cyclic_blade_max_pitch_pos_rad; // ROTOR CYCLIC BLADE MAX PITCH POSITION, Radians
    double disk_pitch_angle_rad;          // DISK PITCH ANGLE, Radians
    double disk_bank_angle_rad;           // DISK BANK ANGLE, Radians
    double disk_coning_pct;               // DISK CONING PCT, Percent
    double rotor_lateral_trim_pct;        // ROTOR LATERAL TRIM PCT, Percent
    double rotor_longitudinal_trim_pct;   // ROTOR LONGITUDINAL TRIM PCT, Percent
    double rotor_rotation_angle_rad;      // ROTOR ROTATION ANGLE:1, Radians
    double elev_trim_pct;                 // ELEVATOR TRIM PCT, Percent
    double ail_trim_pct;                  // AILERON TRIM PCT, Percent
    double rud_trim_pct;                  // RUDDER TRIM PCT, Percent
    double sim_on_ground;                 // SIM ON GROUND, Bool
};

struct DefEntry { const char* name; const char* units; };

constexpr DefEntry kSimVarDefs[] = {
    { "AIRSPEED INDICATED",                  "Knots" },
    { "AIRSPEED TRUE",                       "Knots" },
    { "INCIDENCE ALPHA",                     "Degrees" },
    { "INCIDENCE BETA",                      "Degrees" },
    { "ROTATION VELOCITY BODY X",            "Radians per second" },
    { "ROTATION VELOCITY BODY Y",            "Radians per second" },
    { "ROTATION VELOCITY BODY Z",            "Radians per second" },
    { "G FORCE",                             "GForce" },
    { "VELOCITY WORLD Y",                    "Feet per second" },
    { "VELOCITY BODY X",                     "Feet per second" },
    { "VELOCITY BODY Y",                     "Feet per second" },
    { "VELOCITY BODY Z",                     "Feet per second" },
    { "PLANE PITCH DEGREES",                 "Radians" },
    { "PLANE BANK DEGREES",                  "Radians" },
    { "TOTAL WEIGHT",                        "Pounds" },
    { "AMBIENT DENSITY",                     "Slugs per cubic feet" },
    { "ROTOR RPM:1",                         "Rpm" },
    { "ROTOR RPM:2",                         "Rpm" },
    { "ENG TORQUE PERCENT:1",                "Percent" },
    { "COLLECTIVE POSITION",                 "Percent" },
    { "TAIL ROTOR PEDAL POSITION",           "Percent" },
    { "TAIL ROTOR BLADE PITCH PCT",          "Percent" },
    { "ROTOR COLLECTIVE BLADE PITCH PCT",    "Percent" },
    { "ROTOR CYCLIC BLADE PITCH PCT",        "Percent" },
    { "ROTOR CYCLIC BLADE MAX PITCH POSITION", "Radians" },
    { "DISK PITCH ANGLE",                    "Radians" },
    { "DISK BANK ANGLE",                     "Radians" },
    { "DISK CONING PCT",                     "Percent" },
    { "ROTOR LATERAL TRIM PCT",              "Percent" },
    { "ROTOR LONGITUDINAL TRIM PCT",         "Percent" },
    { "ROTOR ROTATION ANGLE:1",              "Radians" },
    { "ELEVATOR TRIM PCT",                   "Percent" },
    { "AILERON TRIM PCT",                    "Percent" },
    { "RUDDER TRIM PCT",                     "Percent" },
    { "SIM ON GROUND",                       "Bool" },
};

//-----------------------------------------------------------------------------
// State

SOCKET g_socket = INVALID_SOCKET;
sockaddr_in g_dest{};
HANDLE g_sim = nullptr;
uint32_t g_sequence = 0;
char g_udp_host[64] = "127.0.0.1";
uint16_t g_udp_port = 27016;
bool g_running = true;

void DebugLog(const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    OutputDebugStringA(buf);
    fputs(buf, stdout);
    fflush(stdout);
}

//-----------------------------------------------------------------------------
// UDP send

void SendPacket(const SimVarSample& s) {
    if (g_socket == INVALID_SOCKET) return;

    MsfsFfbPacket pkt{};
    pkt.magic = kPacketMagic;
    pkt.version = kPacketVersion;
    pkt.size_bytes = static_cast<uint16_t>(sizeof(MsfsFfbPacket));
    pkt.sequence = g_sequence++;

    pkt.ias_kts                  = static_cast<float>(s.airspeed_indicated_kts);
    pkt.tas_kts                  = static_cast<float>(s.airspeed_true_kts);
    pkt.alpha_deg                = static_cast<float>(s.incidence_alpha_deg);
    pkt.beta_deg                 = static_cast<float>(s.incidence_beta_deg);
    pkt.p_rate_rad_s             = static_cast<float>(s.rotation_velocity_body_x_rad_s);
    pkt.q_rate_rad_s             = static_cast<float>(s.rotation_velocity_body_y_rad_s);
    pkt.r_rate_rad_s             = static_cast<float>(s.rotation_velocity_body_z_rad_s);
    pkt.g_force                  = static_cast<float>(s.g_force);
    pkt.vvi_world_fps            = static_cast<float>(s.velocity_world_y_fps);
    pkt.velocity_body_x_fps      = static_cast<float>(s.velocity_body_x_fps);
    pkt.velocity_body_y_fps      = static_cast<float>(s.velocity_body_y_fps);
    pkt.velocity_body_z_fps      = static_cast<float>(s.velocity_body_z_fps);
    pkt.pitch_rad                = static_cast<float>(s.plane_pitch_rad);
    pkt.bank_rad                 = static_cast<float>(s.plane_bank_rad);
    pkt.total_weight_lb          = static_cast<float>(s.total_weight_lb);
    pkt.ambient_density_slugs_ft3 = static_cast<float>(s.ambient_density_slugs_ft3);
    pkt.main_rotor_rpm           = static_cast<float>(s.main_rotor_rpm);
    pkt.tail_rotor_rpm           = static_cast<float>(s.tail_rotor_rpm);
    pkt.eng_torque_pct           = static_cast<float>(s.eng_torque_percent);
    pkt.collective_pos_pct       = static_cast<float>(s.collective_position_pct);
    pkt.tail_rotor_pedal_pct     = static_cast<float>(s.tail_rotor_pedal_pct);
    pkt.tail_rotor_blade_pitch_pct = static_cast<float>(s.tail_rotor_blade_pitch_pct);
    pkt.rotor_collective_blade_pitch_pct = static_cast<float>(s.rotor_collective_blade_pitch_pct);
    pkt.rotor_cyclic_blade_pitch_pct     = static_cast<float>(s.rotor_cyclic_blade_pitch_pct);
    pkt.rotor_cyclic_blade_max_pitch_pos_rad = static_cast<float>(s.rotor_cyclic_blade_max_pitch_pos_rad);
    pkt.disk_pitch_angle_rad     = static_cast<float>(s.disk_pitch_angle_rad);
    pkt.disk_bank_angle_rad      = static_cast<float>(s.disk_bank_angle_rad);
    pkt.disk_coning_pct          = static_cast<float>(s.disk_coning_pct);
    pkt.rotor_lateral_trim_pct   = static_cast<float>(s.rotor_lateral_trim_pct);
    pkt.rotor_longitudinal_trim_pct = static_cast<float>(s.rotor_longitudinal_trim_pct);
    pkt.rotor_rotation_angle_rad = static_cast<float>(s.rotor_rotation_angle_rad);
    pkt.elev_trim_pct            = static_cast<float>(s.elev_trim_pct);
    pkt.ail_trim_pct             = static_cast<float>(s.ail_trim_pct);
    pkt.rud_trim_pct             = static_cast<float>(s.rud_trim_pct);

    pkt.on_ground = s.sim_on_ground != 0.0 ? 1 : 0;

    sendto(g_socket, reinterpret_cast<const char*>(&pkt), sizeof(pkt), 0,
           reinterpret_cast<const sockaddr*>(&g_dest), sizeof(g_dest));
}

//-----------------------------------------------------------------------------
// SimConnect dispatch callback

void CALLBACK MsfsDispatchCallback(SIMCONNECT_RECV* recv, DWORD /*cbData*/, void* /*ctx*/) {
    switch (recv->dwID) {
    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA: {
        auto* msg = reinterpret_cast<SIMCONNECT_RECV_SIMOBJECT_DATA*>(recv);
        if (msg->dwRequestID == REQ_FFB_SAMPLE && msg->dwDefineCount > 0) {
            const auto* sample = reinterpret_cast<const SimVarSample*>(&msg->dwData);
            SendPacket(*sample);
        }
        break;
    }
    case SIMCONNECT_RECV_ID_QUIT:
        DebugLog("MsfsFfbDataProvider: MSFS quit, exiting.\n");
        g_running = false;
        break;
    case SIMCONNECT_RECV_ID_EXCEPTION: {
        auto* ex = reinterpret_cast<SIMCONNECT_RECV_EXCEPTION*>(recv);
        DebugLog("MsfsFfbDataProvider: SimConnect exception %u (op %u send %u idx %u)\n",
                 ex->dwException, ex->dwID, ex->dwSendID, ex->dwIndex);
        break;
    }
    default:
        break;
    }
}

//-----------------------------------------------------------------------------
// Config loading

void LoadConfig() {
    FILE* file = nullptr;
    fopen_s(&file, "MsfsFfbDataProvider.cfg", "r");
    if (!file) return;

    char line[256];
    while (fgets(line, sizeof(line), file)) {
        if (strncmp(line, "host=", 5) == 0) {
            char host[64] = {};
            if (sscanf_s(line + 5, "%63s", host, (unsigned)_countof(host)) == 1) {
                strncpy_s(g_udp_host, host, _TRUNCATE);
            }
        } else if (strncmp(line, "port=", 5) == 0) {
            unsigned port = 0;
            if (sscanf_s(line + 5, "%u", &port) == 1 && port <= 65535) {
                g_udp_port = static_cast<uint16_t>(port);
            }
        }
    }
    fclose(file);
}

//-----------------------------------------------------------------------------
// Setup

bool InitUdp() {
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        DebugLog("MsfsFfbDataProvider: WSAStartup failed.\n");
        return false;
    }
    g_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_socket == INVALID_SOCKET) {
        DebugLog("MsfsFfbDataProvider: socket() failed.\n");
        return false;
    }
    u_long nonblocking = 1;
    ioctlsocket(g_socket, FIONBIO, &nonblocking);

    g_dest = {};
    g_dest.sin_family = AF_INET;
    g_dest.sin_port = htons(g_udp_port);
    if (inet_pton(AF_INET, g_udp_host, &g_dest.sin_addr) != 1) {
        inet_pton(AF_INET, "127.0.0.1", &g_dest.sin_addr);
    }

    DebugLog("MsfsFfbDataProvider: UDP target = %s:%u, packet = %zu bytes\n",
             g_udp_host, g_udp_port, sizeof(MsfsFfbPacket));
    return true;
}

bool InitSimConnect() {
    HRESULT hr = SimConnect_Open(&g_sim, "DiyFfb MSFS Bridge",
                                 nullptr, 0, nullptr, 0);
    if (FAILED(hr) || g_sim == nullptr) {
        DebugLog("MsfsFfbDataProvider: SimConnect_Open failed (0x%08X).\n", hr);
        return false;
    }
    for (const auto& def : kSimVarDefs) {
        hr = SimConnect_AddToDataDefinition(g_sim, DEF_FFB_SAMPLE,
                                            def.name, def.units,
                                            SIMCONNECT_DATATYPE_FLOAT64);
        if (FAILED(hr)) {
            DebugLog("MsfsFfbDataProvider: AddToDataDefinition '%s' failed (0x%08X).\n",
                     def.name, hr);
            return false;
        }
    }
    hr = SimConnect_RequestDataOnSimObject(g_sim, REQ_FFB_SAMPLE,
                                           DEF_FFB_SAMPLE,
                                           SIMCONNECT_OBJECT_ID_USER,
                                           SIMCONNECT_PERIOD_SIM_FRAME,
                                           SIMCONNECT_DATA_REQUEST_FLAG_DEFAULT,
                                           0, 0, 0);
    if (FAILED(hr)) {
        DebugLog("MsfsFfbDataProvider: RequestDataOnSimObject failed (0x%08X).\n", hr);
        return false;
    }
    return true;
}

BOOL WINAPI ConsoleHandler(DWORD ctrl) {
    if (ctrl == CTRL_C_EVENT || ctrl == CTRL_CLOSE_EVENT ||
        ctrl == CTRL_BREAK_EVENT || ctrl == CTRL_SHUTDOWN_EVENT) {
        g_running = false;
        return TRUE;
    }
    return FALSE;
}

} // namespace

//-----------------------------------------------------------------------------

int main() {
    SetConsoleCtrlHandler(ConsoleHandler, TRUE);
    LoadConfig();

    if (!InitUdp()) {
        return 1;
    }

    DebugLog("MsfsFfbDataProvider: connecting to MSFS via SimConnect... "
             "(retries every 2 s until MSFS is running)\n");
    while (g_running) {
        if (InitSimConnect()) break;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    if (!g_running) {
        if (g_socket != INVALID_SOCKET) closesocket(g_socket);
        WSACleanup();
        return 0;
    }

    DebugLog("MsfsFfbDataProvider: connected to MSFS, streaming.\n");
    while (g_running) {
        SimConnect_CallDispatch(g_sim, MsfsDispatchCallback, nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (g_sim) SimConnect_Close(g_sim);
    if (g_socket != INVALID_SOCKET) closesocket(g_socket);
    WSACleanup();
    DebugLog("MsfsFfbDataProvider: shutdown complete.\n");
    return 0;
}
