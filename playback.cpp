#define PORT_TM 60501

#include <fstream>
#include <stdlib.h>
#include <unistd.h>

#include "UDPSender.hpp"
#include "Telemetry.hpp"

int speed_factor = 2;

int main(int argc, char *argv[])
{
    switch(argc) {
        case 3:
            speed_factor = atoi(argv[2]);
        case 2:
            break;
        default:
            std::cerr << "Calling sequence: playback <filename> [speed factor]\n";
            return 1;
    }

    TelemetrySender telSender("127.0.0.1", PORT_TM);

    std::streampos cur;

    uint8_t buffer[TELEMETRY_PACKET_MAX_SIZE];
    buffer[0] = 0x90;

    uint16_t length;

    TelemetryPacket tp(NULL);

    std::ifstream ifs(argv[1]);

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
                    usleep(1000);
                }

                ifs.seekg(cur);
            }
        }

    }

    return 0;
}
