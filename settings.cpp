#include <cstdio>

#include "settings.hpp"

using namespace std;

struct settings current_settings;
bool settings_changed;

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
        fclose(infile);
        settings_changed = false;
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
        fclose(outfile);
        settings_changed = true;
    }
}
