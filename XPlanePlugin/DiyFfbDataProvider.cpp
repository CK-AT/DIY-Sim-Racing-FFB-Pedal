// Based on https://developer.x-plane.com/code-sample/motionplatformdata/


/*
Plugin to provide FFB relevant data to the DIY FFB SimHub Plugin via UDP.
*/

// has to be included first! X-Plane plugin SDK headers pull in windows.h, which leads to compiler errors
#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

// Globals.
// Use DFFB_ as a prefix for the global variables

// Datarefs
XPLMDataRef	DFFB_DR_ias_kts = NULL;
XPLMDataRef	DFFB_DR_tas_mps = NULL;
XPLMDataRef	DFFB_DR_alpha = NULL;
XPLMDataRef	DFFB_DR_beta = NULL;
XPLMDataRef	DFFB_DR_p = NULL;
XPLMDataRef	DFFB_DR_q = NULL;
XPLMDataRef	DFFB_DR_r = NULL;
XPLMDataRef	DFFB_DR_elev_def = NULL;
XPLMDataRef	DFFB_DR_ail_def = NULL;
XPLMDataRef	DFFB_DR_rud_def = NULL;
XPLMDataRef	DFFB_DR_elev_trim = NULL;
XPLMDataRef	DFFB_DR_ail_trim = NULL;
XPLMDataRef	DFFB_DR_rud_trim = NULL;
XPLMDataRef	DFFB_DR_g_nrml = NULL;
XPLMDataRef	DFFB_DR_on_ground = NULL;
XPLMDataRef DFFB_DR_elev_trim_overr = NULL;
XPLMDataRef DFFB_DR_ail_trim_overr = NULL;
XPLMDataRef DFFB_DR_rud_trim_overr = NULL;
XPLMDataRef DFFB_DR_torque = NULL;
XPLMDataRef DFFB_DR_omega = NULL;
XPLMDataRef DFFB_DR_prop_ratio = NULL;

WSADATA wsaData;
SOCKET sendSocket = INVALID_SOCKET;
static char g_udp_host[64] = "127.0.0.1";
static uint16_t g_udp_port = 27015;
static uint32_t g_udp_send_errors = 0;
static uint32_t g_udp_send_wouldblock = 0;
static float g_udp_last_log_time = 0.0f;

//---------------------------------------------------------------------------
// Function prototypes

float DFFB_DataLoopCB(float elapsedMe, float elapsedSim, int counter, void * refcon);
void DFFB_CalculateMotionData(void);
void DFFB_LoadConfig(void);

#pragma pack(push, 1)
static const uint8_t kMaxRotors = 4;

struct FfbDataPacket {
    uint32_t magic;
    uint16_t version;
    uint16_t size_bytes;
    uint32_t sequence;
    float ias_kts;
    float tas_mps;
    float alpha_deg;
    float beta_deg;
    float p_rate;
    float q_rate;
    float r_rate;
    float elev_def_deg;
    float ail_def_deg;
    float rud_def_deg;
    float elev_trim_deg;
    float ail_trim_deg;
    float rud_trim_deg;
    float g_nrml;
    float torque_nm[kMaxRotors];
    float omega_rad[kMaxRotors];
    float prop_ratio[kMaxRotors];
    uint8_t on_ground;
    uint8_t reserved[3];
};
#pragma pack(pop)

static const uint32_t kPacketMagic = 0x46464244; // "DFFB"
static const uint16_t kPacketVersion = 2;
static const char* kUdpConfigFile = "DiyFfbDataProvider.cfg";
static uint32_t g_udp_sequence = 0;


//---------------------------------------------------------------------------
// SDK Mandatory Callbacks

PLUGIN_API int XPluginStart(
						char *		outName,
						char *		outSig,
						char *		outDesc)
{
	strcpy(outName, "Diy FFB Data Provider");
	strcpy(outSig, "diyffb.dataprovider");

	XPLMRegisterFlightLoopCallback(DFFB_DataLoopCB, 0.02, NULL);
	
	DFFB_DR_ias_kts = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed");
	DFFB_DR_tas_mps = XPLMFindDataRef("sim/flightmodel/position/true_airspeed");
	DFFB_DR_alpha = XPLMFindDataRef("sim/flightmodel/position/alpha");
	DFFB_DR_beta = XPLMFindDataRef("sim/flightmodel/position/beta");
	DFFB_DR_p = XPLMFindDataRef("sim/flightmodel/position/P");
	DFFB_DR_q = XPLMFindDataRef("sim/flightmodel/position/Q");
	DFFB_DR_r = XPLMFindDataRef("sim/flightmodel/position/R");
	DFFB_DR_elev_def = XPLMFindDataRef("sim/flightmodel/controls/elv1_def");
	DFFB_DR_ail_def = XPLMFindDataRef("sim/flightmodel/controls/ail1_def");
	DFFB_DR_rud_def = XPLMFindDataRef("sim/flightmodel/controls/rud1_def");
	DFFB_DR_elev_trim = XPLMFindDataRef("sim/flightmodel/controls/elv_trim");
	DFFB_DR_ail_trim = XPLMFindDataRef("sim/flightmodel/controls/ail_trim");
	DFFB_DR_rud_trim = XPLMFindDataRef("sim/flightmodel/controls/rud_trim");
	DFFB_DR_g_nrml = XPLMFindDataRef("sim/flightmodel/forces/g_nrml");
	DFFB_DR_on_ground = XPLMFindDataRef("sim/flightmodel/failures/onground_any");
    DFFB_DR_elev_trim_overr = XPLMFindDataRef("sim/operation/override/override_pitch_trim");
    DFFB_DR_ail_trim_overr = XPLMFindDataRef("sim/operation/override/override_roll_trim");
    DFFB_DR_rud_trim_overr = XPLMFindDataRef("sim/operation/override/override_yaw_trim");
    DFFB_DR_torque = XPLMFindDataRef("sim/flightmodel/engine/POINT_drag_TRQ");
    DFFB_DR_omega = XPLMFindDataRef("sim/flightmodel/engine/POINT_tacrad");
    DFFB_DR_prop_ratio = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_ratio");
    

	DFFB_LoadConfig();

    int res = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (res != NO_ERROR) {
        snprintf(outDesc, 255, "WSAStartup failed with error %d\n", res);
        return 0;
    }
    
    sendSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sendSocket == INVALID_SOCKET) {
        snprintf(outDesc, 255, "socket failed with error %d\n", WSAGetLastError());
        return 0;
    }
    u_long nonBlocking = 1;
    if (ioctlsocket(sendSocket, FIONBIO, &nonBlocking) != 0) {
        snprintf(outDesc, 255, "ioctlsocket failed with error %d\n", WSAGetLastError());
        closesocket(sendSocket);
        sendSocket = INVALID_SOCKET;
        return 0;
    }

	strcpy(outDesc, "Diy FFB Data Provider: sends FFB-relevant datarefs via UDP.");
    return 1;
}

//---------------------------------------------------------------------------

PLUGIN_API void	XPluginStop(void)
{
	XPLMUnregisterFlightLoopCallback(DFFB_DataLoopCB, NULL);
    closesocket(sendSocket);
    WSACleanup();
}


//---------------------------------------------------------------------------

PLUGIN_API int XPluginEnable(void)
{
	return 1;
}

//---------------------------------------------------------------------------

PLUGIN_API void XPluginDisable(void)
{
}

//---------------------------------------------------------------------------

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void * inParam)
{
}


//---------------------------------------------------------------------------
// FlightLoop callback to read FFB-relevant data forward it via UDP

float DFFB_DataLoopCB(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	DFFB_CalculateMotionData();

	return (float)0.02;
}

//---------------------------------------------------------------------------
// This is original Xplane code converted to use 
// our datarefs instead of the Xplane variables

void DFFB_CalculateMotionData(void)
{
	if (sendSocket == INVALID_SOCKET) {
		return;
	}

	FfbDataPacket packet = {};
	packet.magic = kPacketMagic;
	packet.version = kPacketVersion;
	packet.size_bytes = sizeof(FfbDataPacket);
	packet.sequence = g_udp_sequence++;
	packet.ias_kts = DFFB_DR_ias_kts ? XPLMGetDataf(DFFB_DR_ias_kts) : 0.0f;
	packet.tas_mps = DFFB_DR_tas_mps ? XPLMGetDataf(DFFB_DR_tas_mps) : 0.0f;
	packet.alpha_deg = DFFB_DR_alpha ? XPLMGetDataf(DFFB_DR_alpha) : 0.0f;
	packet.beta_deg = DFFB_DR_beta ? XPLMGetDataf(DFFB_DR_beta) : 0.0f;
	packet.p_rate = DFFB_DR_p ? XPLMGetDataf(DFFB_DR_p) : 0.0f;
	packet.q_rate = DFFB_DR_q ? XPLMGetDataf(DFFB_DR_q) : 0.0f;
	packet.r_rate = DFFB_DR_r ? XPLMGetDataf(DFFB_DR_r) : 0.0f;
	packet.elev_def_deg = DFFB_DR_elev_def ? XPLMGetDataf(DFFB_DR_elev_def) : 0.0f;
	packet.ail_def_deg = DFFB_DR_ail_def ? XPLMGetDataf(DFFB_DR_ail_def) : 0.0f;
	packet.rud_def_deg = DFFB_DR_rud_def ? XPLMGetDataf(DFFB_DR_rud_def) : 0.0f;
	packet.elev_trim_deg = DFFB_DR_elev_trim ? XPLMGetDataf(DFFB_DR_elev_trim) : 0.0f;
    XPLMSetDatai(DFFB_DR_elev_trim_overr, 1);
	packet.ail_trim_deg = DFFB_DR_ail_trim ? XPLMGetDataf(DFFB_DR_ail_trim) : 0.0f;
    XPLMSetDatai(DFFB_DR_ail_trim_overr, 1);
    packet.rud_trim_deg = DFFB_DR_rud_trim ? XPLMGetDataf(DFFB_DR_rud_trim) : 0.0f;
    XPLMSetDatai(DFFB_DR_rud_trim_overr, 1);
    packet.g_nrml = DFFB_DR_g_nrml ? XPLMGetDataf(DFFB_DR_g_nrml) : 0.0f;
    if (DFFB_DR_torque) {
        int count = XPLMGetDatavf(DFFB_DR_torque, packet.torque_nm, 0, kMaxRotors);
        for (int idx = count; idx < kMaxRotors; ++idx) {
            packet.torque_nm[idx] = 0.0f;
        }
    } else {
        memset(packet.torque_nm, 0, sizeof(packet.torque_nm));
    }
    if (DFFB_DR_omega) {
        int count = XPLMGetDatavf(DFFB_DR_omega, packet.omega_rad, 0, kMaxRotors);
        for (int idx = count; idx < kMaxRotors; ++idx) {
            packet.omega_rad[idx] = 0.0f;
        }
    } else {
        memset(packet.omega_rad, 0, sizeof(packet.omega_rad));
    }
    if (DFFB_DR_prop_ratio) {
        int count = XPLMGetDatavf(DFFB_DR_prop_ratio, packet.prop_ratio, 0, kMaxRotors);
        for (int idx = count; idx < kMaxRotors; ++idx) {
            packet.prop_ratio[idx] = 0.0f;
        }
    } else {
        memset(packet.prop_ratio, 0, sizeof(packet.prop_ratio));
    }
	packet.on_ground = DFFB_DR_on_ground ? (XPLMGetDatai(DFFB_DR_on_ground) != 0) : 0;

	struct sockaddr_in ClientAddr;
	int clientAddrSize = (int)sizeof(ClientAddr);
	ClientAddr.sin_family = AF_INET;
	ClientAddr.sin_port = htons(g_udp_port);
	ClientAddr.sin_addr.s_addr = inet_addr(g_udp_host);
	if (ClientAddr.sin_addr.s_addr == INADDR_NONE) {
		ClientAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	}

    int sendResult = sendto(sendSocket, reinterpret_cast<const char*>(&packet), sizeof(packet), 0,
                            (SOCKADDR*)&ClientAddr, clientAddrSize);
    if (sendResult == SOCKET_ERROR) {
        int err = WSAGetLastError();
        if (err == WSAEWOULDBLOCK) {
            g_udp_send_wouldblock++;
        } else {
            g_udp_send_errors++;
        }
    }

    if ((g_udp_send_errors > 0 || g_udp_send_wouldblock > 0) &&
        (XPLMGetElapsedTime() - g_udp_last_log_time) >= 5.0f) {
        char logLine[256] = {};
        snprintf(logLine, sizeof(logLine),
                 "DiyFfbDataProvider: UDP send errors=%u wouldblock=%u (dest=%s:%u)\n",
                 g_udp_send_errors, g_udp_send_wouldblock, g_udp_host, g_udp_port);
        XPLMDebugString(logLine);
        g_udp_last_log_time = XPLMGetElapsedTime();
    }
}

void DFFB_LoadConfig(void)
{
	char prefsPath[512] = {};
	XPLMGetPrefsPath(prefsPath);
	if (prefsPath[0] == '\0') {
		return;
	}

	char configPath[640] = {};
	snprintf(configPath, sizeof(configPath), "%s%s", prefsPath, kUdpConfigFile);

	FILE* file = fopen(configPath, "r");
	if (!file) {
		return;
	}

	char line[256] = {};
	while (fgets(line, sizeof(line), file)) {
		if (strncmp(line, "host=", 5) == 0) {
			char host[64] = {};
			if (sscanf(line + 5, "%63s", host) == 1) {
				strncpy(g_udp_host, host, sizeof(g_udp_host) - 1);
				g_udp_host[sizeof(g_udp_host) - 1] = '\0';
			}
		} else if (strncmp(line, "port=", 5) == 0) {
			unsigned int port = 0;
			if (sscanf(line + 5, "%u", &port) == 1 && port <= 65535) {
				g_udp_port = static_cast<uint16_t>(port);
			}
		}
	}

	fclose(file);
}

//---------------------------------------------------------------------------

