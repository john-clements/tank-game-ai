#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "unistd.h"
#include "time.h"

#include "ncurses.h"

#include "main.h"

#define STATUS_LINES        2       // Lines reserved below frame for text

#define FRAME_PERIOD_US     30000   // 33 FPS

uint64_t g_tg_time_us = 0;

void tg_time_start()
{
    struct timespec tms;

    clock_gettime(CLOCK_REALTIME, &tms);

    g_tg_time_us = (uint64_t)tms.tv_sec * 1000000 + (uint64_t)tms.tv_nsec / 1000;
}

void tg_wait()
{
    struct timespec tms;

    clock_gettime(CLOCK_REALTIME, &tms);

    uint64_t time = (uint64_t)tms.tv_sec * 1000000 + (uint64_t)tms.tv_nsec / 1000;

    if (time >= g_tg_time_us + FRAME_PERIOD_US)
        return;

    usleep(FRAME_PERIOD_US - (time - g_tg_time_us));
}

void get_screen_limits(int* x, int* y)
{
    getmaxyx(stdscr, *y, *x);

    *y = *y - STATUS_LINES - 1;
}

void render_obj(tg_obj* obj)
{
    int y = (int)obj->y;
    int x = (int)obj->x;

    attron(COLOR_PAIR(obj->id) | A_REVERSE);
    for (int row = 0; row < obj->height; row++)
    {
        for (int col = 0; col < obj->width; col++)
            mvaddch(y + row, x + col, ' ');
    }
    attroff(COLOR_PAIR(obj->id) | A_REVERSE);
}

int g_tg_text_col[STATUS_LINES] = {0};
void tg_text_reset()
{
    for (int i = 0; i < STATUS_LINES; i++)
        g_tg_text_col[i] = 0;
}

void tg_draw_text(int line, const char *fmt, ...)
{
    if (line >= STATUS_LINES)
        return;

    char buf[256];

    int max_y, max_x;
    get_screen_limits(&max_x, &max_y);

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    attron(COLOR_PAIR(2));

    mvprintw(max_y + line + 1, g_tg_text_col[line] + 2, "%s", buf);

    attroff(COLOR_PAIR(2));

    g_tg_text_col[line] = g_tg_text_col[line] + len;
}

void tg_init()
{
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    idlok(stdscr, FALSE);
    idcok(stdscr, FALSE);
}

void tg_start_engine(tg_ctx* ctx)
{
    start_color();
    init_pair(1, COLOR_WHITE, COLOR_BLACK);   // Border
    init_pair(2, COLOR_CYAN,  COLOR_BLACK);   // Status text

    for (int i = 0; i < ctx->obj_list_cnt; i++)
        init_pair(ctx->obj_list[i].id, ctx->obj_list[i].color, ctx->obj_list[i].color);

    init_tg_ai(ctx);

    while (1) {
        tg_time_start();

        // Handle input
        int ch = getch();
        if (ch == 'q' || ch == 'Q') break;
 
        for (int i = 0; i < ctx->obj_list_cnt; i++)
        {
            if (ctx->obj_list[i].manual_process == 0)
                tg_obj_process(&ctx->obj_list[i]);
        }

        step_tg_ai(ctx);

        int max_y, max_x;
        get_screen_limits(&max_x, &max_y);

        // Draw
        erase();
 
        // Frame border
        for (int col = 0; col < max_x; col++) {
            mvaddch(0,      col, ACS_HLINE);
            mvaddch(max_y,  col, ACS_HLINE);
        }
        for (int row = 0; row <= max_y; row++) {
            mvaddch(row, 0,         ACS_VLINE);
            mvaddch(row, max_x - 1, ACS_VLINE);
        }
        mvaddch(0,      0,         ACS_ULCORNER);
        mvaddch(0,      max_x - 1, ACS_URCORNER);
        mvaddch(max_y,  0,         ACS_LLCORNER);
        mvaddch(max_y,  max_x - 1, ACS_LRCORNER);

        // Draw label
        attron(COLOR_PAIR(1));
        mvprintw(0, (max_x - 22) / 2, " AI Square  [q] quit ");
        attroff(COLOR_PAIR(1));
 
        // Draw objects
        for (int i = 0; i < ctx->obj_list_cnt; i++)
            render_obj(&ctx->obj_list[i]);

        // Draw status text
        attron(COLOR_PAIR(2));
        draw_tg_ai_status(ctx);
        attroff(COLOR_PAIR(2));

        refresh();
        tg_wait();
    }
 
    endwin();

    free_tg_ai(ctx);
}