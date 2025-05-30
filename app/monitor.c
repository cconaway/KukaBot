#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <signal.h>

#ifdef _WIN32
#   include <windows.h>
#   define SLEEP_MS(ms)   Sleep(ms)
#else
#   include <unistd.h>
#   define SLEEP_MS(ms)   usleep((ms) * 1000)
#endif

#include "kuka_rsi.h"

static volatile bool g_exit = false;
static void on_signal(int sig) { (void)sig; g_exit = true; }

/**
 * Mandatory callback for RSI data.
 * Even if unused, it must be registered to return IPOC every 4ms.
 */
static void on_data_callback(const RSI_CartesianPosition* cart,
                             const RSI_JointPosition* joint,
                             void* user_data) {
    // Optional: could use this callback to log, analyze, or stream data
    // For now, do nothing. It's just required to complete the handshake.
    (void)cart;
    (void)joint;
    (void)user_data;
}

/* -------------------------------------------------------------------------- */
int main(void)
{
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    RSI_Config cfg = {0};
    cfg.local_ip   = "0.0.0.0";    // listen on all interfaces
    cfg.local_port = 59152;        // KUKA RSI default port
    cfg.timeout_ms = 1000;         // timeout if no data
    cfg.verbose    = true;         // ENABLE verbose debug output

    printf("Initializing RSI with configuration:\n");
    printf("  Local IP: %s\n", cfg.local_ip);
    printf("  Local Port: %d\n", cfg.local_port);
    printf("  Timeout: %d ms\n", cfg.timeout_ms);
    printf("  Verbose mode: %s\n", cfg.verbose ? "Enabled" : "Disabled");

    if (RSI_Init(&cfg) != RSI_SUCCESS) {
        fprintf(stderr, "RSI_Init failed\n"); return 1;
    }
    printf("RSI initialized successfully\n");

    // Register mandatory callback
    printf("Registering callbacks...\n");
    if (RSI_SetCallbacks(on_data_callback, NULL, NULL) != RSI_SUCCESS) {
        fprintf(stderr, "RSI_SetCallbacks failed\n");
        RSI_Cleanup(); return 1;
    }
    printf("Callbacks registered successfully\n");

    printf("Starting RSI communication...\n");
    RSI_Error err = RSI_Start();
    if (err != RSI_SUCCESS) {
        fprintf(stderr, "RSI_Start failed with error code: %d\n", err);
        // Add more details about the error if possible
        const char* error_msg = "Unknown error";
        switch(err) {
            case RSI_ERROR_SOCKET_FAILED:
                error_msg = "Socket creation failed";
                break;
            case RSI_ERROR_THREAD_FAILED:
                error_msg = "Error Thread Failed";
                break;
            case RSI_ERROR_INVALID_PARAM:
                error_msg = "Invalid Params";
                break;
            case RSI_ERROR_TIMEOUT:
                error_msg = "Connection timeout";
                break;
            case RSI_ERROR_ALREADY_RUNNING:
                error_msg = "Already Running";
                break;
            case RSI_ERROR_NOT_RUNNING:
                error_msg = "Not Running";
                break;
            case RSI_ERROR_UNKNOWN:
                error_msg = "Unknown error";
                break;
            // Add more cases if there are other specific error codes
        }
        fprintf(stderr, "Error details: %s\n", error_msg);
        RSI_Cleanup(); return 1;
    }

    puts("RSI monitor ready …  (Ctrl-C to quit)");

    RSI_CartesianPosition cart = {0};
    RSI_JointPosition     joint = {0};
    RSI_Statistics        stats = {0};
    uint32_t              last_ipoc = 0;

    while (!g_exit)
    {
        if (RSI_GetCartesianPosition(&cart) == RSI_SUCCESS &&
            RSI_GetJointPosition(&joint)   == RSI_SUCCESS &&
            RSI_GetStatistics(&stats)      == RSI_SUCCESS)
        {
            if (cart.ipoc != last_ipoc) {
                last_ipoc = cart.ipoc;

                printf(
                    "IPOC %6u | "
                    "XYZ %.1f %.1f %.1f mm | "
                    "ABC %.1f %.1f %.1f ° | "
                    "A %.1f %.1f %.1f %.1f %.1f %.1f ° | "
                    "pkt_rx %llu  late>4ms %llu\r",
                    cart.ipoc,
                    cart.x, cart.y, cart.z,
                    cart.a, cart.b, cart.c,
                    joint.axis[0], joint.axis[1], joint.axis[2],
                    joint.axis[3], joint.axis[4], joint.axis[5],
                    (unsigned long long)stats.packets_received,
                    (unsigned long long)stats.late_responses
                );
                fflush(stdout);
            }
        }

    }

    puts("\nStopping …");
    RSI_Stop();
    RSI_Cleanup();
    puts("Done.");
    return 0;
}