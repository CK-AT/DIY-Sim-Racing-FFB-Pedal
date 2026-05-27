// DataRef Logger — logs rotor and flight state datarefs to CSV at ~20 Hz.
// Standalone X-Plane plugin for vibration model development.
// Based on DiyFfbDataProvider.cpp structure.

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "XPLMProcessing.h"
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

//-----------------------------------------------------------------------------
// Dataref table — add/remove entries here to change what gets logged.
// Entries with array_index >= 0 read a float array element.
// Entries with array_index == -1 read a scalar float.
// Entries with array_index == -2 read a scalar int (logged as float).

struct DataRefEntry {
    const char *path;
    const char *label;       // short CSV column name
    int         array_index; // -1=scalar float, -2=scalar int, >=0=array float[N]
    XPLMDataRef ref;
};

static DataRefEntry g_refs[] = {
    // Flight state
    { "sim/cockpit2/gauges/indicators/airspeed_kts_pilot", "ias_kts",       -1, NULL },
    { "sim/flightmodel/position/true_airspeed",            "tas_mps",       -1, NULL },
    { "sim/flightmodel/position/alpha",                    "alpha_deg",     -1, NULL },
    { "sim/flightmodel/position/beta",                     "beta_deg",      -1, NULL },
    { "sim/flightmodel/forces/g_nrml",                     "g_nrml",        -1, NULL },
    { "sim/flightmodel/forces/g_axil",                     "g_axil",        -1, NULL },
    { "sim/flightmodel/forces/g_side",                     "g_side",        -1, NULL },
    { "sim/flightmodel/position/P",                        "P_rad_s",       -1, NULL },
    { "sim/flightmodel/position/Q",                        "Q_rad_s",       -1, NULL },
    { "sim/flightmodel/position/R",                        "R_rad_s",       -1, NULL },
    { "sim/flightmodel/position/theta",                    "pitch_deg",     -1, NULL },
    { "sim/flightmodel/position/phi",                      "roll_deg",      -1, NULL },
    { "sim/flightmodel/position/vh_ind_fpm",               "vvi_fpm",       -1, NULL },
    { "sim/flightmodel/position/groundspeed",              "gs_mps",        -1, NULL },
    { "sim/flightmodel/failures/onground_any",             "on_ground",     -2, NULL },

    // Aero torques
    { "sim/flightmodel/forces/L_aero",                     "L_aero",        -1, NULL },
    { "sim/flightmodel/forces/M_aero",                     "M_aero",        -1, NULL },
    { "sim/flightmodel/forces/N_aero",                     "N_aero",        -1, NULL },

    // Rotor RPM (main + tail)
    { "sim/cockpit2/engine/indicators/prop_speed_rpm",     "rpm_main",       0, NULL },
    { "sim/cockpit2/engine/indicators/prop_speed_rpm",     "rpm_tail",       1, NULL },

    // Rotor torque (main + tail)
    { "sim/flightmodel/engine/POINT_drag_TRQ",             "torque_main",    0, NULL },
    { "sim/flightmodel/engine/POINT_drag_TRQ",             "torque_tail",    1, NULL },

    // Controls
    { "sim/joystick/yoke_pitch_ratio",                     "yoke_pitch",    -1, NULL },
    { "sim/joystick/yoke_roll_ratio",                      "yoke_roll",     -1, NULL },
    { "sim/joystick/yoke_heading_ratio",                   "yoke_yaw",      -1, NULL },
    { "sim/cockpit2/engine/actuators/prop_ratio",          "collective",     0, NULL },

    // Environment
    { "sim/weather/temperature_ambient_c",                 "temp_c",        -1, NULL },
    { "sim/flightmodel/misc/h_ind",                        "alt_ft",        -1, NULL },

    // === Vibration envelope drivers (all verified in DataRefs.txt) ===

    // 1/rev: per-axis blade alpha — proxy for flapping (scales with IAS)
    { "sim/flightmodel/cyclic/cyclic_elev_blad_alph",           "blade_alph_pitch", 0, NULL },
    { "sim/flightmodel/cyclic/cyclic_ailn_blad_alph",           "blade_alph_roll",  0, NULL },

    // 2/rev high-speed: blade slap ratio
    { "sim/flightmodel2/engines/rotor_blade_slap_rat",          "slap_rat",         0, NULL },

    // 2/rev ETL: vortex ring state (0.50 hover → 0.25 forward flight)
    { "sim/flightmodel/engine/vortex_ring_state",               "vrs_0",            0, NULL },

    // 3/rev RBS: blade alpha approaching stall
    { "sim/flightmodel2/engines/rotor_blade_alpha_deg",         "blade_alpha",      0, NULL },

    // Context signals
    { "sim/flightmodel2/engines/rotor_disc_alpha_deg",          "disc_alpha",       0, NULL },
    { "sim/flightmodel2/engines/propwash_mtr_sec",              "propwash_mps",     0, NULL },
    { "sim/flightmodel2/engines/rotor_radius_mtr",              "rotor_radius",     0, NULL },

    // Disc tilt from cyclic input (pilot input, not vibration)
    { "sim/flightmodel2/engines/rotor_cyclic_elevator_tilt_deg","cycli_pitch",      0, NULL },
    { "sim/flightmodel2/engines/rotor_cyclic_aileron_tilt_deg", "cycli_roll",       0, NULL },

    // Rotor moments (small, noisy — logged for reference)
    { "sim/flightmodel/forces/Q_rotor_rad",                     "Q_rotor",          0, NULL },
    { "sim/flightmodel/forces/R_rotor_rad",                     "R_rotor",          0, NULL },

    // === Plan 17 §4.1 additions: parallel the MSFS SimConnect surface ===
    // and enable propwash/VRS derivation calibration.
    { "sim/flightmodel/weight/m_total",                         "m_total_kg",      -1, NULL },
    { "sim/flightmodel/position/local_vy",                      "local_vy_mps",    -1, NULL },
    { "sim/weather/rho",                                        "rho",             -1, NULL },
    // Disc plane attitude including flapping — closer match to MSFS DISK PITCH/BANK ANGLE
    // than the cyclic-input-only cycli_pitch/cycli_roll already logged. Names are
    // TBD against DataRefs.txt; logger gracefully reports NOT FOUND at startup if absent.
    { "sim/flightmodel2/engines/rotor_disc_pitch_deg",          "disc_pitch_actual_deg", 0, NULL },
    { "sim/flightmodel2/engines/rotor_disc_roll_deg",           "disc_roll_actual_deg",  0, NULL },
    // Direct rotor angular velocity (rad/s) — avoids RPM rounding for calibration.
    { "sim/flightmodel/engine/POINT_tacrad",                    "omega_rad_s",      0, NULL },
};

static const int NUM_REFS = sizeof(g_refs) / sizeof(g_refs[0]);
static FILE *g_log_file = NULL;

//-----------------------------------------------------------------------------

float DRLOG_FlightLoopCB(float elapsedMe, float elapsedSim, int counter, void *refcon);

//-----------------------------------------------------------------------------

PLUGIN_API int XPluginStart(char *outName, char *outSig, char *outDesc)
{
    strcpy(outName, "DataRef Logger");
    strcpy(outSig, "diyffb.dataref_logger");
    strcpy(outDesc, "Logs rotor datarefs to CSV for vibration model development.");

    // Resolve all datarefs
    for (int i = 0; i < NUM_REFS; i++) {
        g_refs[i].ref = XPLMFindDataRef(g_refs[i].path);
        if (!g_refs[i].ref) {
            char buf[256];
            snprintf(buf, sizeof(buf), "DataRefLogger: NOT FOUND: %s\n", g_refs[i].path);
            XPLMDebugString(buf);
        }
    }

    return 1;
}

PLUGIN_API void XPluginStop(void)
{
}

PLUGIN_API int XPluginEnable(void)
{
    // Open log file
    char sysPath[512];
    XPLMGetSystemPath(sysPath);

    char logPath[640];
    snprintf(logPath, sizeof(logPath), "%sOutput/dataref_log.csv", sysPath);
    g_log_file = fopen(logPath, "w");

    if (g_log_file) {
        // Write header
        fprintf(g_log_file, "time_s");
        for (int i = 0; i < NUM_REFS; i++) {
            fprintf(g_log_file, ",%s", g_refs[i].label);
        }
        fprintf(g_log_file, "\n");

        char buf[640];
        snprintf(buf, sizeof(buf), "DataRefLogger: logging to %s\n", logPath);
        XPLMDebugString(buf);
    } else {
        char buf[640];
        snprintf(buf, sizeof(buf), "DataRefLogger: FAILED to open %s\n", logPath);
        XPLMDebugString(buf);
    }

    XPLMRegisterFlightLoopCallback(DRLOG_FlightLoopCB, 0.05f, NULL);
    return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    XPLMUnregisterFlightLoopCallback(DRLOG_FlightLoopCB, NULL);

    if (g_log_file) {
        fclose(g_log_file);
        g_log_file = NULL;
        XPLMDebugString("DataRefLogger: closed log file\n");
    }
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFrom, int inMsg, void *inParam)
{
}

//-----------------------------------------------------------------------------

float DRLOG_FlightLoopCB(float elapsedMe, float elapsedSim, int counter, void *refcon)
{
    if (!g_log_file) return 0.05f;

    // Timestamp
    fprintf(g_log_file, "%.4f", XPLMGetElapsedTime());

    for (int i = 0; i < NUM_REFS; i++) {
        float val = 0.0f;
        if (g_refs[i].ref) {
            if (g_refs[i].array_index >= 0) {
                XPLMGetDatavf(g_refs[i].ref, &val, g_refs[i].array_index, 1);
            } else if (g_refs[i].array_index == -2) {
                val = (float)XPLMGetDatai(g_refs[i].ref);
            } else {
                val = XPLMGetDataf(g_refs[i].ref);
            }
        }
        fprintf(g_log_file, ",%.6f", val);
    }
    fprintf(g_log_file, "\n");

    return 0.05f;  // 20 Hz
}
