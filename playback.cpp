#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "UDPSender.hpp"
#include "Telemetry.hpp"

void help_message(char name[])
{
    std::cout << "Usage: " << name << " [options] <files>\n";
    std::cout << "Command-line options:\n";
    std::cout << "-i<ip>     Send telemetry packets to this IP instead of 127.0.0.1\n";
    std::cout << "-p<port>   Send telemetry packets to this port instead of 60501\n";
    std::cout << "-s<speed>  Integer multiplier to speed up playback\n";
}

int main(int argc, char *argv[])
{
    char ip[20];
    strncpy(ip, "127.0.0.1", 20);

    uint16_t port = 60501;
    uint16_t speed_factor = 1;

    uint16_t nfiles = 0;

    for(int i = 1; i < argc; i++) {
        if(argv[i][0] == '-') {
            for(int j = 1; argv[i][j] != 0; j++) {
                switch(argv[i][j]) {
                    case 'i':
                        strncpy(ip, &argv[i][j+1], 20);
                        ip[19] = 0;
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 'p':
                        port = atoi(&argv[i][j+1]);
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 's':
                        speed_factor = atoi(&argv[i][j+1]);
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

    std::cout << "Playing back " << nfiles << " file(s) to " << ip << ":" << port;
    std::cout << " at " << speed_factor << "x speed\n";

    if(nfiles == 0) {
        help_message(argv[0]);
        return -1;
    }

    TelemetrySender telSender(ip, port);

    std::streampos cur;

    uint8_t buffer[TELEMETRY_PACKET_MAX_SIZE];
    buffer[0] = 0x90;

    uint16_t length;

    TelemetryPacket tp(NULL);

    for(int i = 1; i < argc; i++) {
        if(argv[i][0] != '-') {
            std::cout << "Playing back " << argv[i] << std::endl;

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

                            if(tp.valid()) {
                                telSender.send(&tp);
                                if(((uint64_t)cur+length+15)/(size/50) > cur/(size/50)) {
                                    std::cout << ".";
                                    std::cout.flush();
                                }
                                usleep(30000); // FIXME: play back at actual rate
                            }

                            ifs.seekg(cur);
                        }
                    }
                }

                std::cout << std::endl;

                ifs.close();
            }
        }
    }

    return 0;
}
