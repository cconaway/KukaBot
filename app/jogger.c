/*  jogger.c  – simple RSI jog utility
*
*  ──────────────────────────────────────────────────────────────
*  Keys              Increment (mm)                Axis
*  -------------------------------------------------------------
*    w / s              +STEP  / –STEP               Z (up / down)
*    a / d              –STEP  / +STEP               X (left / right)
*    q / e              –STEP  / +STEP               Y (back / fwd)
*    space              zero all corrections
*    Esc / Ctrl-C       quit
*  ──────────────────────────────────────────────────────────────
*
*  Build  (mac / Linux) :  gcc key_jogger.c ../src/kuka_rsi.c -I../include \
*                          -o key_jogger -pthread
*  Build  (Windows)     :  cl /EHsc key_jogger.c ..\src\kuka_rsi.c \
*                          /I..\include ws2_32.lib
*
*  Run                  :  ./key_jogger
*/

#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdbool.h>
#include <signal.h>
#include <string.h>

#ifdef _WIN32
#   include <conio.h>
#   include <windows.h>
#   define KBHIT()      _kbhit()
#   define GETCH()      _getch()
#   define SLEEP_MS(ms) Sleep(ms)
#else                  /* POSIX ------------------------------------------------*/
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

/* -------------------------------------------------------------------------- */
static volatile bool g_exit = false;
static void on_signal(int sig) { (void)sig; g_exit = true; }

static void zero_correction(RSI_CartesianCorrection* c) {
    memset(c, 0, sizeof(*c));
}

int main(void)
{
    /* 1.  Initialise RSI --------------------------------------------------- */
    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    RSI_Config cfg = { .local_ip = "0.0.0.0",
                    .local_port = 59152,
                    .timeout_ms = 1000,
                    .verbose = false };
    if (RSI_Init(&cfg) != RSI_SUCCESS)               { puts("RSI_Init failed");  return 1; }
    if (RSI_Start()   != RSI_SUCCESS)                { puts("RSI_Start failed"); RSI_Cleanup(); return 1; }

    const double STEP = 5.0;                         /* 2 mm jog step */
    RSI_CartesianCorrection corr;  zero_correction(&corr);
    RSI_CartesianPosition   pos   = {0};

    puts("Keyboard jogger ready –  press Esc or Ctrl-C to quit.");

    /* 2.  Main loop -------------------------------------------------------- */
    while (!g_exit)
    {
        /* --- read position (optional – just for display) ------------------ */
        if (RSI_GetCartesianPosition(&pos) == RSI_SUCCESS) {
            printf("\rIPOC %6u  XYZ %.1f %.1f %.1f mm   ", pos.ipoc,
                pos.x, pos.y, pos.z);
            fflush(stdout);
        }

        /* --- keyboard handling ------------------------------------------- */
        if (KBHIT()) {
            int ch = GETCH();
            switch (ch)
            {
                case 'w': corr.z +=  STEP; break;   /* up    */
                case 's': corr.z -=  STEP; break;   /* down  */
                case 'a': corr.x -=  STEP; break;   /* left  */
                case 'd': corr.x +=  STEP; break;   /* right */
                case 'q': corr.y -=  STEP; break;   /* back  */
                case 'e': corr.y +=  STEP; break;   /* fwd   */
                case ' ': zero_correction(&corr);  break;   /* zero  */
                case 27 : g_exit = true;           break;   /* Esc   */
                default : break;
            }
        }

        /* --- send current correction ------------------------------------- */
        RSI_SetCartesianCorrection(&corr);

        /* zero the correction so it is one-shot (jog-style) */
        zero_correction(&corr);

        SLEEP_MS(4);   /* 4 ms ≈ RSI cycle */
    }

    /* 3.  Shutdown --------------------------------------------------------- */
    puts("\nStopping …");
    RSI_Stop();
    RSI_Cleanup();
    puts("Done.");
    return 0;
}
