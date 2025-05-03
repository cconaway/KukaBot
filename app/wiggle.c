/* wiggle.c – KUKA RSI automatic X-wiggle, 8 mm peak-to-peak
 * 2025-05-03  (zero-pulse edition)
 *---------------------------------------------------------------------*
 *  • Captures start-of-program X and oscillates ±4 mm about it.        *
 *  • Sends a “pulse” pair each time:  ±0.05 mm   →   0 mm.             *
 *    └─ The zero on the second cycle prevents run-on after the step.   *
 *  • 12 ms cadence  →  ≈4 mm s-¹ feed rate.                            *
 *  • Esc key (or Ctrl-C) aborts.                                       *
 *---------------------------------------------------------------------*/

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#   include <conio.h>
#   include <windows.h>
#   define KBHIT()      _kbhit()
#   define GETCH()      _getch()
#   define SLEEP_MS(ms) Sleep(ms)
#else
#   include <unistd.h>
#   include <termios.h>
#   include <fcntl.h>
#   include <sys/select.h>
static int kbhit(void) {
    struct timeval tv = {0};
    fd_set         fds;
    FD_ZERO(&fds); FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}
static int getch_nonblock(void) {
    struct termios oldt, newt;
    int            ch = -1;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}
#   define KBHIT()      kbhit()
#   define GETCH()      getch_nonblock()
#   define SLEEP_MS(ms) usleep((ms)*1000)
#endif

#include "kuka_rsi.h"

/*─ Global exit flag ─*/
static volatile bool g_exit = false;
static void on_signal(int sig) { (void)sig; g_exit = true; }

/*─ Dummy mandatory callback ─*/
static void on_data(const RSI_CartesianPosition* c,
                    const RSI_JointPosition*    j,
                    void*                       ud)
{ (void)c; (void)j; (void)ud; }

/*─ Helper ─*/
static void zero_correction(RSI_CartesianCorrection* c)
{ memset(c, 0, sizeof(*c)); }

int main(void)
{
    /*──────────────────── 1.  RSI start-up ────────────────────*/
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    RSI_Config cfg = {
        .local_ip   = "0.0.0.0",
        .local_port = 59152,
        .timeout_ms = 1000,
        .verbose    = true
    };

    puts("Starting RSI 8 mm wiggle (zero-pulse edition) …");
    if (RSI_Init(&cfg) != RSI_SUCCESS ||
        RSI_SetCallbacks(on_data, NULL, NULL) != RSI_SUCCESS ||
        RSI_Start() != RSI_SUCCESS)
    {
        fprintf(stderr, "RSI start-up failed\n");
        RSI_Cleanup();
        return 1;
    }

    /*──────────────────── 2.  Motion parameters ───────────────*/
    const double STEP_MM   = 0.1;   /* one pulse = 0.05 mm */
    const double TRAVEL_MM = 1.0;    /* ±4 mm about start    */
    const int    LOOP_MS   = 0;     /* ≈3 RSI cycles        */
    const double EPS_MM    = STEP_MM / 2.0;  /* flip buffer   */

    RSI_CartesianPosition pos  = {0};
    RSI_CartesianCorrection corr, zero_corr;
    zero_correction(&corr);
    zero_correction(&zero_corr);

    double start_x      = NAN;
    double lower_limit  = 0.0;
    double upper_limit  = 0.0;
    bool   dir_positive = false;  /* start toward –X */
    bool   pending_zero = false;  /* track pulse phase */

    puts("→ Motion begins automatically.  Press Esc or Ctrl-C to quit.");

    /*──────────────────── 3.  Main loop ───────────────────────*/
    while (!g_exit) {

        if (RSI_GetCartesianPosition(&pos) == RSI_SUCCESS) {

            if (isnan(start_x)) {            /* latch reference */
                start_x     = pos.x;

                lower_limit = 441.0;;
                upper_limit = 449.0;
                printf("Reference X: %.3f mm  →  [%.3f … %.3f]\n",
                       start_x, lower_limit, upper_limit);
            }

            /*── Decide correction to send this cycle ──*/
            if (pending_zero) {
                /* second half of pulse: zero */
                /*RSI_SetCartesianCorrection(&zero_corr); */
                pending_zero = false;
            } else {
                /* first half: ±STEP_MM, then tag for zero next time */
                if (pos.x < lower_limit) {
                    dir_positive = true;
                    printf("At lower limit: %.3f\n", lower_limit);
                }
                else if (pos.x > upper_limit) {
                    dir_positive = false;
                    printf("At upper limit: %.3f\n", upper_limit);
                }

                corr.x = dir_positive ? STEP_MM : -STEP_MM;
                RSI_SetCartesianCorrection(&corr);
                pending_zero = true;
            }

            /*── Telemetry ──*/
            printf("\rIPOC %-10u | X = %7.3f mm | %s",
                   pos.ipoc,
                   pos.x,
                   pending_zero ? "STEP " : "ZERO");
            fflush(stdout);
        }

        /* Esc key abort */
        if (KBHIT() && GETCH() == 27) {
            puts("\nEsc pressed – exiting.");
            g_exit = true;
        }

        SLEEP_MS(LOOP_MS);
    }

    /*──────────────────── 4.  Shutdown ────────────────────────*/
    RSI_SetCartesianCorrection(&zero_corr);  /* make sure we leave zeroed */
    puts("\nStopping …");
    RSI_Stop();
    RSI_Cleanup();
    puts("Done.");
    return 0;
}
