#include <cstdio>

#include "settings.hpp"

using namespace std;

struct settings current_settings;
uint8_t current_table;

int load_settings(uint8_t table)
{
    char filename[10];
    sprintf(filename, "table.%03d", table);

    FILE *infile = fopen(filename, "r");
    if(infile != NULL) {
        fscanf(infile, "%hhu", &current_settings.PY_rate);
        fscanf(infile, "%hhu", &current_settings.PY_gain);
        fscanf(infile, "%hu", &current_settings.PY_exposure);
        fscanf(infile, "%hhu", &current_settings.R_rate);
        fscanf(infile, "%hhu", &current_settings.R_gain);
        fscanf(infile, "%hu", &current_settings.R_exposure);
        fscanf(infile, "%hhu", &current_settings.cadence_housekeeping);
        fscanf(infile, "%hhu", &current_settings.cadence_a2d);
        fscanf(infile, "%hhu", &current_settings.cadence_science);
        fscanf(infile, "%f", &current_settings.screen_center_x);
        fscanf(infile, "%f", &current_settings.screen_center_y);
        fscanf(infile, "%f", &current_settings.screen_rotation);
        fclose(infile);
        current_table = table;
        return 0;
    } else return -1;
}

void save_settings()
{
    FILE *outfile = fopen("table.255", "w");
    if(outfile != NULL) {
        fprintf(outfile, "%hhu\n", current_settings.PY_rate);
        fprintf(outfile, "%hhu\n", current_settings.PY_gain);
        fprintf(outfile, "%hu\n", current_settings.PY_exposure);
        fprintf(outfile, "%hhu\n", current_settings.R_rate);
        fprintf(outfile, "%hhu\n", current_settings.R_gain);
        fprintf(outfile, "%hu\n", current_settings.R_exposure);
        fprintf(outfile, "%hhu\n", current_settings.cadence_housekeeping);
        fprintf(outfile, "%hhu\n", current_settings.cadence_a2d);
        fprintf(outfile, "%hhu\n", current_settings.cadence_science);
        fprintf(outfile, "%f\n", current_settings.screen_center_x);
        fprintf(outfile, "%f\n", current_settings.screen_center_y);
        fprintf(outfile, "%f\n", current_settings.screen_rotation);
        fclose(outfile);
        current_table = 255;
    }
}
