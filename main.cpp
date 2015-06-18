#define MAX_THREADS 30
#define LOG_PACKETS false

#define SAVE_LOCATION "./"

//Sleep settings (microseconds)
#define USLEEP_KILL            3000000 // how long to wait before terminating threads
#define USLEEP_TM_SEND            5000 // period for popping off the telemetry queue
#define USLEEP_UDP_LISTEN         1000 // safety measure in case UDP listening is changed to non-blocking
#define USLEEP_MAIN               5000 // period for checking for new commands in the queue
#define USLEEP_IRS              100000 // cadence for checking IR sensor

//IP addresses
#define IP_FC "192.168.2.100"
#define IP_LOOPBACK "127.0.0.1"
#define IP_TM IP_FC //default IP address unless overridden on the command line

//UDP ports, aside from PORT_IMAGE, which is TCP
#define PORT_CMD      50501 // commands, FC (receive)
#define PORT_TM       60501 // send telemetry to FC

//Acknowledgement error codes
#define ACK_NOERROR 0x00
#define ACK_BADCRC  0x03
#define ACK_BADSYS  0x04
#define ACK_BADCOM  0x05
#define ACK_BADVALUE 0x11

//GRIPS system ID
#define SYS_ID_FC  0x00
#define SYS_ID_ASP 0x40
#define SYS_ID_PYC 0x4A
#define SYS_ID_RC  0x4B
#define SYS_ID_IRS 0x4C

//GRIPS telemetry types
#define TM_ACK          0x01
#define TM_HOUSEKEEPING 0x02
#define TM_SETTINGS     0x03
#define TM_A2D          0x04
#define TM_SCIENCE      0x10
#define TM_IMAGE        0x20

//GRIPS commands, shared
#define KEY_NULL                 0x00

//GRIPS commands, ASP specific
#define KEY_OTHER                0x10

#include <cstring>
#include <stdio.h>      /* for printf() and fprintf() */
#include <pthread.h>    /* for multithreading */
#include <stdlib.h>     /* for atoi() and exit() */
#include <signal.h>     /* for signal() */
#include <math.h>
#include <ctime>        /* time_t, struct tm, time, gmtime */
#include <iostream>
#include <string>
#include <fstream>

#include "UDPSender.hpp"
#include "UDPReceiver.hpp"
#include "Command.hpp"
#include "Telemetry.hpp"
#include "types.hpp"

#include "main.hpp"
#include "camera.hpp"
#include "oeb.h"
#include "dmm.h"

// global declarations
uint8_t command_sequence_number = -1;
uint8_t latest_system_id = 0xFF;
uint8_t latest_command_key = 0xFF;
volatile uint8_t py_image_counter = 0;
volatile uint8_t roll_image_counter = 0;
float grid_rotation_rate = -1;
struct dmminfo DMM1;
float temp_py = 0, temp_roll = 0, temp_mb = 0;
uint8_t cadence_housekeeping = 1, cadence_a2d = 1, cadence_science = 1; //seconds
char ip_tm[20];

// global mode variables
bool MODE_COMPRESS = false; //used by camera main
bool MODE_MOCK = false; //used by camera main
bool MODE_NETWORK = false;
bool MODE_TIMING = false; //used by camera main
bool MODE_UNCONNECTED = false;
bool MODE_VERBOSE = false; //used by camera main

// UDP packet queues
TelemetryPacketQueue tm_packet_queue; //for sending
CommandPacketQueue cm_packet_queue; //for receiving

// related to threads
bool stop_message[MAX_THREADS];
pthread_t threads[MAX_THREADS];
bool started[MAX_THREADS];
int tid_listen = -1; //Stores the ID for the CommandListener thread
pthread_mutex_t mutexStartThread; //Keeps new threads from being started simultaneously
pthread_mutex_t mutexAnalysis; //For copying results from image analysis

struct Thread_data{
    int thread_id;
    uint8_t system_id;
    uint8_t command_key;
    uint8_t payload_size;
    uint8_t payload[15];
};
struct Thread_data thread_data[MAX_THREADS];

sig_atomic_t volatile g_running = 1;
sig_atomic_t volatile g_running_camera_main = 1;

//Function declarations
void sig_handler(int signum);

void start_thread(void *(*start_routine) (void *), const Thread_data *tdata);
void start_all_workers();
void kill_all_threads(); //kills all threads
void kill_all_workers(); //kills all threads except the one that listens for commands

void *TelemetrySenderThread(void *threadargs);
void *TelemetryHousekeepingThread(void *threadargs);
void *TelemetryA2DThread(void *threadargs);
void *TelemetryScienceThread(void *threadargs);

void *CommandListenerThread(void *threadargs);
void cmd_process_command(CommandPacket &cp);
void *CommandHandlerThread(void *threadargs);
void queue_cmd_proc_ack_tmpacket( uint8_t error_code, uint64_t response );
void queue_settings_tmpacket();

void *IRSensorThread(void *threadargs);

void *CameraMainThread(void *threadargs);

template <class T>
bool set_if_different(T& variable, T value); //returns true if the value is different

void sig_handler(int signum)
{
    if ((signum == SIGINT) || (signum == SIGTERM))
    {
        if (signum == SIGINT) std::cerr << "Keyboard interrupt received\n";
        if (signum == SIGTERM) std::cerr << "Termination signal received\n";
        g_running = 0;
        g_running_camera_main = 0;
    }
}

template <class T>
bool set_if_different(T& variable, T value)
{
    if(variable != value) {
        variable = value;
        return true;
    } else return false;
}

// Forces a sleep through any interrupts
// This is necessary because PvAPI creates a 1-second SIGALRM timer
// http://stackoverflow.com/questions/13865166/cant-pause-a-thread-in-conjunction-with-third-party-library
int usleep_force(uint64_t microseconds)
{
    struct timespec amount, remaining;
    amount.tv_nsec = (microseconds % 1000000) * 1000;
    amount.tv_sec = microseconds / 1000000;
    while (nanosleep(&amount, &remaining) == -1) {
        amount = remaining;
    }
    return 0;
}

void make_histo(uint8_t histo[], float histogram[])
{
    for(int i = 0; i < 16; i++) {
        float total1 = 0, total2 = 0;
        for(int j = 0; j < 8; j++) {
            total1 += histogram[i * 16 + j];
            total2 += histogram[i * 16 + j + 8];
        }
        uint8_t first = (total1 > 0 ? 2 * log10(total1) + 13.2 : 0);
        uint8_t second = (total2 > 0 ? 2 * log10(total2) + 13.2 : 0);
        histo[i] = (second << 4) + first;
    }
}

void kill_all_workers()
{
    for(int i = 0; i < MAX_THREADS; i++ ){
        if ((i != tid_listen) && started[i]) {
            stop_message[i] = true;
        }
    }
    g_running_camera_main = 0;
    usleep_force(USLEEP_KILL);
    for(int i = 0; i < MAX_THREADS; i++ ){
        if ((i != tid_listen) && started[i]) {
            printf("Quitting thread %i, quitting status is %i\n", i, pthread_cancel(threads[i]));
            started[i] = false;
        }
    }
}

void kill_all_threads()
{
    if (started[tid_listen]) {
        stop_message[tid_listen] = true;
        kill_all_workers();
        if (started[tid_listen]) {
            printf("Quitting thread %i, quitting status is %i\n", tid_listen, pthread_cancel(threads[tid_listen]));
            started[tid_listen] = false;
        }
    }
}

void *TelemetrySenderThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetrySender thread #%ld!\n", tid);

/*
    char timestamp[14];
    char filename[128];
    std::ofstream log;

    if (LOG_PACKETS) {
        writeCurrentUT(timestamp);
        sprintf(filename, "%slog_tm_%s.bin", SAVE_LOCATION, timestamp);
        filename[128 - 1] = '\0';
        printf("Creating telemetry log file %s \n",filename);
        log.open(filename, std::ofstream::binary);
    }
*/
    TelemetrySender telSender(ip_tm, (unsigned short) PORT_TM);

    while(!stop_message[tid])
    {
        usleep_force(USLEEP_TM_SEND);

        if( !tm_packet_queue.empty() ){
            TelemetryPacket tp(NULL);
            tm_packet_queue >> tp;
            telSender.send( &tp );
            if(MODE_NETWORK) std::cout << "TelemetrySender: " << tp.getLength() << " bytes, " << tp << std::endl;
/*
            if (LOG_PACKETS && log.is_open()) {
                uint16_t length = tp.getLength();
                uint8_t *payload = new uint8_t[length];
                tp.outputTo(payload);
                log.write((char *)payload, length);
                delete payload;
                //log.flush();
            }
*/
        }
    }

    printf("TelemetrySender thread #%ld exiting\n", tid);
/*
    if (LOG_PACKETS && log.is_open()) log.close();
*/
    started[tid] = false;
    pthread_exit( NULL );
}

void *TelemetryHousekeepingThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetryHousekeeping thread #%ld!\n", tid);

    uint32_t tm_frame_sequence_number = 0;

    while(!stop_message[tid])
    {
        usleep_force(cadence_housekeeping * 1000000);
        tm_frame_sequence_number++;

        TelemetryPacket tp(SYS_ID_ASP, TM_HOUSEKEEPING, tm_frame_sequence_number, oeb_get_clock());

        uint8_t status_bitfield = 0;
        bitwrite(&status_bitfield, 0, 1, CAMERAS[0].Handle != NULL);
        bitwrite(&status_bitfield, 1, 1, CAMERAS[1].Handle != NULL);
        tp << status_bitfield << latest_command_key;

        tp << (int16_t)(temp_py * 100) << (int16_t)(temp_roll * 100) << (int16_t)(temp_mb * 100);

        tm_packet_queue << tp;
    }

    printf("TelemetryHousekeeping thread #%ld exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}

void *TelemetryA2DThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetryA2D thread #%ld!\n", tid);

    uint32_t tm_frame_sequence_number = 0;

    InitDMM1(); //FIXME: this thread needs to stop if the DMM A2D board isn't present

    while(!stop_message[tid])
    {
        usleep_force(cadence_a2d * 1000000);

        DMMUpdateADC(&DMM1);

        tm_frame_sequence_number++;
        TelemetryPacket tp(SYS_ID_ASP, TM_A2D, tm_frame_sequence_number, oeb_get_clock());

        for (int i = 0; i < 32; i++) {
            tp << (uint16_t)DMM1.ain[i];
        }

        tm_packet_queue << tp;
    }

    printf("TelemetryA2D thread #%ld exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}

void *TelemetryScienceThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetryScience thread #%ld!\n", tid);

    uint32_t tm_frame_sequence_number = 0;
    float old_grid_orientation = 0;

    while(!stop_message[tid])
    {
        usleep_force(cadence_science * 1000000);
        tm_frame_sequence_number++;

        TelemetryPacket tp(SYS_ID_ASP, TM_SCIENCE, tm_frame_sequence_number, oeb_get_clock());

        pthread_mutex_lock(&mutexAnalysis);

        uint8_t quality_bitfield = 0;
        bitwrite(&quality_bitfield, 0, 1, PY_ANALYSIS.there[0]);
        bitwrite(&quality_bitfield, 1, 1, PY_ANALYSIS.there[1]);
        bitwrite(&quality_bitfield, 2, 1, PY_ANALYSIS.there[2]);
        tp << quality_bitfield;

        uint8_t count1 = py_image_counter;
        uint8_t count2 = roll_image_counter;
        tp << count1 << count2;
        py_image_counter = 0;
        roll_image_counter = 0;

        uint8_t num_fiducials = 0;
        tp << num_fiducials;

        float offset_pitch = 0, uncert_pitch = 0;
        float offset_yaw = 0, uncert_yaw = 0;
        tp << offset_pitch << uncert_pitch << offset_yaw << uncert_yaw;

        float new_grid_orientation = 0;
        float delta_grid_orientation = new_grid_orientation - old_grid_orientation;
        if (delta_grid_orientation < 0) delta_grid_orientation += 360.;
        tp << new_grid_orientation << delta_grid_orientation;
        old_grid_orientation = new_grid_orientation;

        tp << grid_rotation_rate;

        uint8_t py_histo[16];
        make_histo(py_histo, PY_ANALYSIS.histogram);
        tp.append_bytes(py_histo, 16);

        uint8_t roll_histo[16];
        make_histo(roll_histo, R_ANALYSIS.histogram);
        tp.append_bytes(roll_histo, 16);

        //the three Sun centers in pixel coordinates
        for (int i = 0; i < 3; i++) {
            tp << (uint16_t)(PY_ANALYSIS.xp[i] * 10) << (uint16_t)(PY_ANALYSIS.yp[i] * 10);
        }

        uint16_t fiducial_x[4], fiducial_y[4];
        memset(fiducial_x, 0, 4 * sizeof(uint16_t));
        memset(fiducial_y, 0, 4 * sizeof(uint16_t));
        for (int i = 0; i < 4; i++) {
            tp << (uint16_t)(fiducial_x[i] * 10) << (uint16_t)(fiducial_y[i] * 10);
        }

        pthread_mutex_unlock(&mutexAnalysis);

        tm_packet_queue << tp;
    }

    printf("TelemetryScience thread #%ld exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}

void *CommandListenerThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("CommandListener thread #%ld!\n", tid);

    tid_listen = tid;

    CommandReceiver comReceiver( (unsigned short) PORT_CMD);
    comReceiver.init_connection();

    while(!stop_message[tid])
    {
        unsigned int packet_length;

        usleep_force(USLEEP_UDP_LISTEN);
        packet_length = comReceiver.listen( );
        printf("CommandListenerThread: %i bytes, ", packet_length);
        uint8_t *packet;
        packet = new uint8_t[packet_length];
        comReceiver.get_packet( packet );

        CommandPacket command_packet( packet, packet_length );

        if (command_packet.valid()){
            printf("valid checksum, ");

            command_sequence_number = command_packet.getCounter();

            // update the command count
            printf("command sequence number %i", command_sequence_number);

            cm_packet_queue << command_packet;
        } else {
            printf("INVALID checksum");
            queue_cmd_proc_ack_tmpacket(ACK_BADCRC, 0xFFFFFFFF);
        }
        printf("\n");

        delete packet;
    }

    printf("CommandListener thread #%ld exiting\n", tid);
    comReceiver.close_connection();
    started[tid] = false;
    pthread_exit( NULL );
}

void queue_cmd_proc_ack_tmpacket( uint8_t error_code, uint64_t response )
{
    TelemetryPacket ack_tp(SYS_ID_ASP, TM_ACK, command_sequence_number, oeb_get_clock());
    ack_tp << error_code << response;
    tm_packet_queue << ack_tp;
}

void queue_settings_tmpacket()
{
    static uint16_t counter = 0;
    TelemetryPacket tp(SYS_ID_ASP, TM_SETTINGS, counter, oeb_get_clock());

    uint16_t last_parameter_table = 0;
    tp << last_parameter_table;

    uint8_t py_fps = CAMERAS[0].Rate;
    uint8_t py_gain = CAMERAS[0].Gain;
    uint16_t py_exposure = CAMERAS[0].ExposureLength;
    tp << py_fps << py_gain << py_exposure;

    uint8_t roll_fps = CAMERAS[1].Rate;
    uint8_t roll_gain = CAMERAS[1].Gain;
    uint16_t roll_exposure = CAMERAS[1].ExposureLength;
    tp << roll_fps << roll_gain << roll_exposure;

    tm_packet_queue << tp;
}

void *CommandHandlerThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    struct Thread_data *my_data;
    uint8_t error_code = 0xFF; //command not implemented
    uint64_t response = 0;
    my_data = (struct Thread_data *) threadargs;

    uint64_t value = 0;

    switch(my_data->system_id)
    {
        case SYS_ID_ASP:
            switch(my_data->command_key)
            {
                case 0x99: //Set clock value to sync to
                    memcpy(&value, my_data->payload, 6);
                    oeb_set_clock(value);
                    error_code = 0;
                    break;
                case 0xA0: //Turn OFF pitch-yaw camera
                    break;
                case 0xA1: //Turn ON pitch-yaw camera
                    break;
                case 0xB0: //Turn OFF roll camera
                    break;
                case 0xB1: //Turn ON roll camera
                    break;
                case 0xC0: //Turn OFF IR sensor
                    break;
                case 0xC1: //Turn ON IR sensor
                    break;
                case 0xD0: //Request settings telemetry packet
                    queue_settings_tmpacket();
                    error_code = 0;
                    break;
                case 0xD1: //Set cadence of housekeeping packet
                    value = *(uint8_t *)(my_data->payload);
                    if(value > 0) {
                        cadence_housekeeping = value;
                        std::cout << "Setting cadence of housekeeping packet to "
                                  << (int)cadence_housekeeping << " s\n";
                        error_code = 0;
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case 0xD2: //Set cadence of A2D temperatures packet
                    value = *(uint8_t *)(my_data->payload);
                    if(value > 0) {
                        cadence_a2d = value;
                        std::cout << "Setting cadence of A2D temperatures packet to "
                                  << (int)cadence_a2d << " s\n";
                        error_code = 0;
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case 0xD3: //Set cadence of science packet
                    value = *(uint8_t *)(my_data->payload);
                    if(value > 0) {
                        cadence_science = value;
                        std::cout << "Setting cadence of science packet to "
                                  << (int)cadence_science << " s\n";
                        error_code = 0;
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case 0xE0: //Load parameter table
                    break;
                case 0xF0: //Restart worker threads, handled earlier
                case 0xF1: //Restart all threads, handled earlier
                case 0xF2: //Restart runtime, handled earlier
                case 0xFF: //Graceful computer shutdown, handled earlier
                default:
                    std::cerr << "Unknown command\n";
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            break;
        case SYS_ID_PYC:
            switch(my_data->command_key & 0xF)
            {
                case 0x4: //Set FPS
                    value = *(uint16_t *)(my_data->payload);
                    if(value > 0) {
                        CAMERAS[0].Rate = value;
                        std::cout << "Setting pitch-yaw camera rate to " << CAMERAS[0].Rate << " Hz\n";
                        error_code = arm_timer();
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case 0x5: //Set gain
                    CAMERAS[0].Gain = *(uint8_t *)(my_data->payload);
                    std::cout << "Setting pitch-yaw camera gain to " << CAMERAS[0].Gain << " dB\n";
                    break;
                case 0x6: //Set exposure
                    CAMERAS[0].ExposureLength = *(uint16_t *)(my_data->payload);
                    std::cout << "Setting pitch-yaw camera exposure to " << CAMERAS[0].ExposureLength << " us\n";
                    break;
                case 0xC: //Send latest image
                    TRANSMIT_NEXT_PY_IMAGE = true;
                    error_code = 0;
                    break;
                case 0xD: //Send specific image
                    break;
                default:
                    std::cerr << "Unknown command\n";
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            break;
        case SYS_ID_RC:
            switch(my_data->command_key & 0xF)
            {
                case 0x4: //Set FPS
                    value = *(uint16_t *)(my_data->payload);
                    if(value > 0) {
                        CAMERAS[0].Rate = value;
                        std::cout << "Setting row camera rate to " << CAMERAS[1].Rate << " Hz\n";
                        error_code = arm_timer();
                    } else {
                        error_code = ACK_BADVALUE;
                    }
                    break;
                case 0x5: //Set gain
                    CAMERAS[1].Gain = *(uint8_t *)(my_data->payload);
                    std::cout << "Setting roll camera gain to " << CAMERAS[1].Gain << " dB\n";
                    break;
                case 0x6: //Set exposure
                    CAMERAS[1].ExposureLength = *(uint16_t *)(my_data->payload);
                    std::cout << "Setting roll camera exposure to " << CAMERAS[1].ExposureLength << " us\n";
                    break;
                case 0xC: //Send latest image
                    TRANSMIT_NEXT_R_IMAGE = true;
                    error_code = 0;
                    break;
                case 0xD: //Send specific image
                    break;
                default:
                    std::cerr << "Unknown command\n";
                    error_code = ACK_BADCOM; //unknown command
                    response = my_data->command_key;
            } //switch for command key
            break;
        default:
            std::cerr << "Unknown system ID\n";
            error_code = ACK_BADSYS; //unknown system ID
            response = my_data->system_id;
    } //switch for system ID

    queue_cmd_proc_ack_tmpacket( error_code, response );

    started[tid] = false;
    pthread_exit(NULL);
}

void *IRSensorThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("IRSensor thread #%ld!\n", tid);

    uint64_t old_trigger_time = oeb_get_irs();
    uint64_t new_trigger_time = 0;
    uint64_t delta = 1;
    float local_grid_rotation_rate = -1;

    while(!stop_message[tid])
    {
        usleep_force(USLEEP_IRS);
        new_trigger_time = oeb_get_irs();
        if (new_trigger_time > old_trigger_time) {
            delta = new_trigger_time - old_trigger_time;

            //Assuming four pulses per full rotation
            local_grid_rotation_rate = 0.25 / (delta * 1e-7 / 60);

            //Reject anomalous readings
            if (local_grid_rotation_rate < 30) {
                grid_rotation_rate = local_grid_rotation_rate;
                old_trigger_time = new_trigger_time;
            }
        }
    }

    printf("IRSensor thread #%ld exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}

void *CameraMainThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("CameraMain thread #%ld!\n", tid);

    camera_main(); // monitors g_running directly

    printf("CameraMain thread #%ld exiting\n", tid);
    started[tid] = false;
    pthread_exit( NULL );
}

void start_thread(void *(*routine) (void *), const Thread_data *tdata)
{
    pthread_mutex_lock(&mutexStartThread);
    int i = 0;
    while (started[i] == true) {
        i++;
        if (i == MAX_THREADS) return; //should probably thrown an exception
    }

    //Copy the thread data to a global to prevent deallocation
    if (tdata != NULL) memcpy(&thread_data[i], tdata, sizeof(Thread_data));
    thread_data[i].thread_id = i;

    stop_message[i] = false;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

    int rc = pthread_create(&threads[i], &attr, routine, &thread_data[i]);
    if (rc != 0) {
        printf("ERROR; return code from pthread_create() is %d\n", rc);
    } else started[i] = true;

    pthread_attr_destroy(&attr);

    pthread_mutex_unlock(&mutexStartThread);

    return;
}

void cmd_process_command(CommandPacket &cp)
{
    std::cout << cp << std::endl;

    Thread_data tdata;
    memset(&tdata, 0, sizeof(Thread_data));
    tdata.system_id = cp.getSystemID();
    tdata.command_key = cp.getCmdType();
    cp.setReadIndex(8);
    tdata.payload_size = cp.remainingBytes();
    if (tdata.payload_size > 0) {
        cp.readNextTo_bytes(tdata.payload, tdata.payload_size);
    }

    switch(tdata.command_key)
    {
        case 0x00:
            queue_cmd_proc_ack_tmpacket(0, 0);
            break;
        case 0xF0:
            kill_all_workers();
            start_all_workers();
            break;
        case 0xF1:
            kill_all_threads();
            start_thread(CommandListenerThread, NULL);
            start_all_workers();
            break;
        case 0xF2:
            g_running = 0;
            break;
        case 0xFF:
            std::cerr << "Graceful computer shutdown not yet implemented!\n";
            break;
        default:
            start_thread(CommandHandlerThread, &tdata);
    } //switch
}

void start_all_workers()
{
    start_thread(TelemetryHousekeepingThread, NULL);
    if (!MODE_UNCONNECTED) start_thread(TelemetryA2DThread, NULL);
    start_thread(TelemetryScienceThread, NULL);
    start_thread(TelemetrySenderThread, NULL);
    start_thread(IRSensorThread, NULL);

    g_running_camera_main = 1;
    if (!MODE_UNCONNECTED) start_thread(CameraMainThread, NULL);
}

int main(int argc, char *argv[])
{
    strncpy(ip_tm, IP_TM, 20);

    for(int i = 1; i < argc; i++) {
        if(argv[i][0] == '-') {
            for(int j = 1; argv[i][j] != 0; j++) {
                switch(argv[i][j]) {
                    case 'c':
                        std::cout << "Compress mode\n";
                        MODE_COMPRESS = true;
                        break;
                    case 'i':
                        strncpy(ip_tm, &argv[i][j+1], 20);
                        ip_tm[19] = 0;
                        j = strlen(&argv[i][0]) - 1;
                        break;
                    case 'm':
                        std::cout << "Mock mode\n";
                        MODE_MOCK = true;
                        break;
                    case 'n':
                        std::cout << "Network diagnostics mode\n";
                        MODE_NETWORK = true;
                        break;
                    case 't':
                        std::cout << "Timing mode\n";
                        MODE_TIMING = true;
                        break;
                    case 'u':
                        std::cout << "Unconnected mode\n";
                        MODE_UNCONNECTED = true;
                        break;
                    case 'v':
                        std::cout << "Verbose mode\n";
                        MODE_VERBOSE = true;
                        break;
                    case '?':
                        std::cout << "Command-line options:\n";
                        std::cout << "-c      Use Rice compression when saving FITS files\n";
                        std::cout << "-i<ip>  Send telemetry packets to this IP (instead of the FC's IP)\n";
                        std::cout << "-m      Use mock images instead of real images\n";
                        std::cout << "-n      Display network packets (can be crazy!)\n";
                        std::cout << "-t      Perform timing tests\n";
                        std::cout << "-u      Assume the A2D board and cameras are not connected\n";
                        std::cout << "-v      Verbose messages (mostly on the camera side)\n";
                        return -1;
                    default:
                        std::cerr << "Unknown option, use -? to list options\n";
                        return -1;
                }
            }
        }
    }

    // to catch a Ctrl-C or termination signal and clean up
    signal(SIGINT, &sig_handler);
    signal(SIGTERM, &sig_handler);

    // elevate privilege level
    if(iopl(3) != 0) {
        std::cerr << "Need to run with root permissions (e.g., sudo)\n";
        return -1;
    }

    // initialize odds & ends board
    if(oeb_init() != 0) return 1;

    pthread_mutex_init(&mutexStartThread, NULL);
    pthread_mutex_init(&mutexAnalysis, NULL);

    std::cout << "Sending telemetry to " << ip_tm << std::endl;

    /* Create worker threads */
    printf("In main: creating threads\n");

    for(int i = 0; i < MAX_THREADS; i++ ){
        started[0] = false;
    }

    // start the listen for commands thread right away
    start_thread(CommandListenerThread, NULL);
    start_all_workers();

    while(g_running){
        usleep_force(USLEEP_MAIN);

        // check if new commands have been added to command queue and service them
        if (!cm_packet_queue.empty()){
            //printf("size of queue: %zu\n", cm_packet_queue.size());
            CommandPacket cp(NULL);
            cm_packet_queue >> cp;

            latest_system_id = cp.getSystemID();
            latest_command_key = cp.getCmdType();

            printf("Received system ID/command key 0x%02X/0x%02X\n", latest_system_id, latest_command_key);
            cmd_process_command(cp);
        }
    }

    /* Last thing that main() should do */
    printf("Quitting and cleaning up.\n");

    /* wait for threads to finish */
    kill_all_threads();
    pthread_mutex_destroy(&mutexStartThread);
    pthread_mutex_destroy(&mutexAnalysis);

    oeb_uninit();

    pthread_exit(NULL);
    return 0;
}
