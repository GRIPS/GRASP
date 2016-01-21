#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "Telemetry.hpp"

void help_message(char name[])
{
    std::cout << "Usage: " << name << " [options] <files>\n";
    std::cout << "Command-line options:\n";
    std::cout << "-e             Only inspect event packets (TmType 0xF3 from card cages)\n";
    std::cout << "-fs<SystemID>  Only inspect telemetry packets with this SystemID in decimal\n";
    std::cout << "-ft<TmType>    Only inspect telemetry packets with this TmType in decimal\n";
}

int main(int argc, char *argv[])
{
    setbuf(stdout, NULL);

    uint16_t nfiles = 0;

    uint8_t filter_systemid = 0xFF, filter_tmtype = 0xFF;
    bool filter_events = false;

    for(int i = 1; i < argc; i++) {
        if(argv[i][0] == '-') {
            for(int j = 1; argv[i][j] != 0; j++) {
                switch(argv[i][j]) {
                    case 'e':
                        filter_events = true;
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 'f':
                        if(argv[i][j+1] == 's') filter_systemid = atoi(&argv[i][j+2]);
                        if(argv[i][j+1] == 't') filter_tmtype = atoi(&argv[i][j+2]);
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case '?':
                        help_message(argv[0]);
                        return -1;
                    default:
                        std::cerr << "Unknown option, use -? to list options\n";
                        return -1;
                }
            }
        } else {
            nfiles++;
        }
    }

    if(nfiles == 0) {
        help_message(argv[0]);
        return -1;
    }

    std::streampos cur;

    uint8_t buffer[TELEMETRY_PACKET_MAX_SIZE];
    buffer[0] = 0x90;

    uint16_t length;

    uint32_t count[256][256];
    uint32_t bad_checksum[256][256];
    uint64_t amount[256][256];
    Clock first_systemtime = 0, last_systemtime = 0;
    bool not_coincident[16];

    TelemetryPacket tp(NULL);

    for(int i = 1; i < argc; i++) {
        memset(count, 0, sizeof(count));
        memset(bad_checksum, 0, sizeof(bad_checksum));
        memset(amount, 0, sizeof(amount));
        first_systemtime = last_systemtime = 0;
        memset(not_coincident, 0, sizeof(not_coincident));

        if(argv[i][0] != '-') {
            std::cout << "Inspecting " << argv[i] << std::endl;

            uint8_t percent_completed = 0;

            std::ifstream ifs(argv[i]);

            if(ifs.good()) {
                ifs.seekg(0, ifs.end);
                std::streampos size = ifs.tellg();
                ifs.seekg(0, ifs.beg);

                std::cout << size << " bytes: ";
                std::cout.flush();

                while (ifs.good()) {
                    if(ifs.get() == 0x90) {
                        if(ifs.peek() == 0xeb) {
                            cur = ifs.tellg(); // points one byte into sync word
                            ifs.seekg(5, std::ios::cur);
                            ifs.read((char *)&length, 2);

                            if(length > TELEMETRY_PACKET_MAX_SIZE-16) continue; //invalid payload size

                            ifs.seekg(cur);

                            ifs.read((char *)buffer+1, length+15);

                            tp = TelemetryPacket(buffer, length+16);

                            if(tp.valid() &&
                               ((!filter_events) || (((tp.getSystemID() & 0xF0) == 0x80) && (tp.getTmType() == 0xF3))) &&
                               ((filter_systemid == 0xFF) || (tp.getSystemID() == filter_systemid)) &&
                               ((filter_tmtype == 0xFF) || (tp.getTmType() == filter_tmtype))) {

                                count[tp.getSystemID()][tp.getTmType()]++;
                                amount[tp.getSystemID()][tp.getTmType()] += length+16;

                                if(((tp.getSystemID() & 0xF0) == 0x80) && (tp.getTmType() == 0xF3)) {
                                    if(((buffer[19] == 0) && (buffer[20] == 0) && (buffer[21] == 0)) ||
                                       ((buffer[22] == 0) && (buffer[23] == 0) && (buffer[24] == 0))) {
                                        not_coincident[tp.getSystemID() & 0x0F] = true;
                                    }
                                }

                                if(first_systemtime == 0) first_systemtime = tp.getSystemTime();
                                last_systemtime = tp.getSystemTime();
                            } else {
                                bad_checksum[tp.getSystemID()][tp.getTmType()]++;
                            }

                            if((((uint64_t)ifs.tellg())*100/size) > percent_completed) {
                                percent_completed++;
                                printf("%3u%%\b\b\b\b", percent_completed);
                            }

                            ifs.seekg(cur);
                        }
                    }
                }

                ifs.close();

                std::cout << "\nPacket breakdown:\n";
                for(int j = 0; j < 256; j++) {
                    for(int k = 0; k < 256; k++) {
                        if(count[j][k] > 0) {
                            printf("  0x%02x 0x%02x : [%4.1f%%] %lu", j, k, 100. * amount[j][k] / size, count[j][k]);
                            if(((j & 0xF0) == 0x80) && (k == 0xF3) && not_coincident[j & 0x0F]) {
                                printf(" (includes non-coincident data!)");
                            }
                            if(bad_checksum[j][k] > 0) {
                                printf(", plus %lu with bad checksums", bad_checksum[j][k]);
                            }
                            printf("\n");
                        }
                    }
                }

                printf("Elapsed gondola time: %llu (%f minutes)\n\n", last_systemtime - first_systemtime, (last_systemtime - first_systemtime) * 1e-7 / 60);
            }
        }
    }

    return 0;
}
