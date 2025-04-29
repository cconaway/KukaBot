/*  simple_monitor.c  –  lean RSI telemetry reader
*
*  Build   (mac / Linux) :  gcc simple_monitor.c ../src/kuka_rsi.c -I../include -o monitor -pthread
*  Build   (Windows)     :  cl /EHsc simple_monitor.c ..\src\kuka_rsi.c /I..\include ws2_32.lib
*
*  Run                  :  ./monitor          # waits for robot to connect
*  Stop                 :  Ctrl-C
*/

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

/* -------------------------------------------------------------------------- */
int main(void)
{
    /* 1.- basic library config -------------------------------------------- */
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    RSI_Config cfg = {0};
    cfg.local_ip   = "0.0.0.0";      /* listen on all NICs              */
    cfg.local_port = 59152;          /* KUKA’s default RSI port         */
    cfg.timeout_ms = 1000;           /* connection-lost watchdog        */
    cfg.verbose    = false;          /* silence library’s own prints    */

    if (RSI_Init(&cfg) != RSI_SUCCESS) {
        fprintf(stderr,"RSI_Init failed\n"); return 1;
    }
    if (RSI_Start()   != RSI_SUCCESS) {
        fprintf(stderr,"RSI_Start failed\n"); RSI_Cleanup(); return 1;
    }

    puts("RSI monitor ready …  (Ctrl-C to quit)");

    /* 2.- live polling loop ----------------------------------------------- */
    RSI_CartesianPosition cart = {0};
    RSI_JointPosition     joint = {0};
    RSI_Statistics        stats = {0};
    uint32_t              last_ipoc = 0;

    while (!g_exit)
    {
        if (   RSI_GetCartesianPosition(&cart) == RSI_SUCCESS
            && RSI_GetJointPosition(&joint)   == RSI_SUCCESS
            && RSI_GetStatistics(&stats)      == RSI_SUCCESS)
        {
            /* only print when IPOC increments – keeps console readable */
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

        SLEEP_MS(10);          /* -- 100 Hz console refresh            */
    }

    /* 3.- shutdown --------------------------------------------------------- */
    puts("\nStopping …");
    RSI_Stop();
    RSI_Cleanup();
    puts("Done.");
    return 0;
}
