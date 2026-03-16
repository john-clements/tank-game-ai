#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "ncurses.h"
#include "unistd.h"

#include "main.h"

#define TABLE_WIDTH     16
#define DIVIDER_WIDTH   64

#define DIVIDER(str)        print_divider((str), DIVIDER_WIDTH)
#define TABLE(data, size)   print_mem_table(data, size, TABLE_WIDTH)

// Prints divider line
void print_divider(char* title, uint8_t wall_len)
{
    uint32_t    i           = 0;
    uint8_t     title_len   = 0xFF;

    if (title == NULL) 
        title = "";

    do 
    {
        title_len++;
    } while ((title[title_len] != '\0') || (title_len>= 0xFF));

    for (i = 0; i < ((wall_len >> 1) - ((title_len+2) >>1)); i++)
        printf("-");

    if (title == "")
        printf("--");
    else
        printf(" %s ", title);

    for (i = 0; i < ((wall_len >> 1) - ((title_len+2) >>1)); i++)
        printf("-");

    if (!(title_len % 2))
        printf("-");

    printf("\n");
}

// Print the byte contents at a memory address in a table format
void print_mem_table(uint8_t* mem,
                     uint32_t mem_size,
                     uint32_t line_width)
{
    printf("\n  TABLE  |");

    for (int i = 0; i < line_width; i++)
        printf(" %02X", i);

    printf("\n---------|");

    for (int i = 0; i < line_width; i++)
        printf("---");

    printf("\n%08X | ", 0);

    for (int i = 0; i < mem_size; i++)
    {
        printf("%02X ", mem[i]);
        if ((i+1)%line_width == 0)
            printf("\n%08X | ",i+1);
    }

    printf("\n");
}

void print_c_byte_array(uint8_t* mem,
                        uint32_t mem_size,
                        uint32_t line_width)
{
    printf("{\n    ");

    for (int i = 0; i < mem_size; i++)
    {
        // If end of line
        if ((i+1)%line_width == 0)
        {
            // If not last line
            if (i+1 != mem_size)
                printf("0x%02X,\n    ", mem[i]);
            else
                printf("0x%02X\n", mem[i]);
        }
        else if (i+1 == mem_size)
            printf("0x%02X\n", mem[i]);
        else
            printf("0x%02X, ", mem[i]);
    }

    printf("}\n");
}

void main()
{
    DIVIDER("START");

    start_ai_obj_test();

    DIVIDER("END");
}