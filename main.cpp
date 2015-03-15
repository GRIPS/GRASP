#define MAX_THREADS 30
#define LOG_PACKETS false

#define SAVE_LOCATION "./"

//Sleep settings (seconds)
#define SLEEP_KILL             2 // waits when killing all threads

//Sleep settings (microseconds)
#define USLEEP_CMD_SEND           5000 // period for popping off the command queue
#define USLEEP_TM_SEND           50000 // period for popping off the telemetry queue
#define USLEEP_TM_HOUSEKEEPING 1000000 // period for adding housekeeping telemetry packets to queue
#define USLEEP_TM_A2D          1000000 // period for adding A2D telemetry packets to queue
#define USLEEP_TM_SCIENCE      1000000 // period for adding science telemetry packets to queue
#define USLEEP_UDP_LISTEN         1000 // safety measure in case UDP listening is changed to non-blocking
#define USLEEP_MAIN               5000 // period for checking for new commands
#define USLEEP_IRS              100000 // cadence for checking IR sensor

//IP addresses
#define IP_FC      "192.168.2.100"

#define IP_LOOPBACK "127.0.0.1"

//UDP ports, aside from PORT_IMAGE, which is TCP
#define PORT_CMD      50501 // commands, FC (receive)
#define PORT_TM       60501 // send telemetry to FC

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

//GRIPS commands, shared
#define KEY_NULL                 0x00

//GRIPS commands, ASP specific
#define KEY_OTHER                0x10

#include <cstring>
#include <stdio.h>      /* for printf() and fprintf() */
#include <pthread.h>    /* for multithreading */
#include <stdlib.h>     /* for atoi() and exit() */
#include <unistd.h>     /* for sleep()  */
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

#include "oeb.h"
#include "dmm.h"

// global declarations
uint8_t command_sequence_number = -1;
uint8_t latest_system_id = 0xFF;
uint8_t latest_command_key = 0xFF;
uint8_t py_image_counter = 0;
uint8_t roll_image_counter = 0;
float grid_rotation_rate = -1;
struct dmminfo DMM1;
float temp_py = 0, temp_roll = 0, temp_mb = 0;

TelemetryPacketQueue tm_packet_queue; //for sending
CommandPacketQueue cm_packet_queue; //for receiving

// related to threads
bool stop_message[MAX_THREADS];
pthread_t threads[MAX_THREADS];
bool started[MAX_THREADS];
int tid_listen = -1; //Stores the ID for the CommandListener thread
pthread_mutex_t mutexStartThread; //Keeps new threads from being started simultaneously

struct Thread_data{
    int thread_id;
    uint8_t system_id;
    uint8_t command_key;
    uint8_t payload_size;
    uint8_t payload[15];
};
struct Thread_data thread_data[MAX_THREADS];

sig_atomic_t volatile g_running = 1;

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
void queue_cmd_proc_ack_tmpacket( uint64_t error_code );
void queue_settings_tmpacket();

void *IRSensorThread(void *threadargs);

void *GRASPReceiverThread(void *threadargs);

template <class T>
bool set_if_different(T& variable, T value); //returns true if the value is different

void sig_handler(int signum)
{
    if ((signum == SIGINT) || (signum == SIGTERM))
    {
        if (signum == SIGINT) std::cerr << "Keyboard interrupt received\n";
        if (signum == SIGTERM) std::cerr << "Termination signal received\n";
        g_running = 0;
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

void kill_all_workers()
{
    for(int i = 0; i < MAX_THREADS; i++ ){
        if ((i != tid_listen) && started[i]) {
            stop_message[i] = true;
        }
    }
    sleep(SLEEP_KILL);
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
    TelemetrySender telSender(IP_FC, (unsigned short) PORT_TM);

    while(!stop_message[tid])
    {
        usleep(USLEEP_TM_SEND);

        if( !tm_packet_queue.empty() ){
            TelemetryPacket tp(NULL);
            tm_packet_queue >> tp;
            telSender.send( &tp );
            //std::cout << "TelemetrySender:" << tp << std::endl;
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
        usleep(USLEEP_TM_HOUSEKEEPING);
        tm_frame_sequence_number++;

        TelemetryPacket tp(SYS_ID_ASP, TM_HOUSEKEEPING, tm_frame_sequence_number, oeb_get_clock());

        uint8_t status_bitfield = 0;
        #ifndef FAKE_TM

        #else
        status_bitfield = (tm_frame_sequence_number % 2) ? 0x7 : 0x0;
        #endif
        tp << status_bitfield << latest_command_key;

        #ifndef FAKE_TM

        #else
        temp_py = tm_frame_sequence_number;
        temp_roll = tm_frame_sequence_number + 1;
        temp_mb = tm_frame_sequence_number + 2;
        #endif
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

    #ifndef FAKE_TM
    InitDMM1();
    #endif

    while(!stop_message[tid])
    {
        usleep(USLEEP_TM_A2D);

        #ifndef FAKE_TM
        DMMUpdateADC(&DMM1);
        #endif

        tm_frame_sequence_number++;
        TelemetryPacket tp(SYS_ID_ASP, TM_A2D, tm_frame_sequence_number, oeb_get_clock());

        for (int i = 0; i < 32; i++) {
            #ifndef FAKE_TM
            tp << (uint16_t)DMM1.ain[i];
            #else
            tp << (uint16_t)(tm_frame_sequence_number + i);
            #endif
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
        usleep(USLEEP_TM_SCIENCE);
        tm_frame_sequence_number++;

        TelemetryPacket tp(SYS_ID_ASP, TM_SCIENCE, tm_frame_sequence_number, oeb_get_clock());

        uint8_t quality_bitfield = 0;
        tp << quality_bitfield;

        tp << py_image_counter << roll_image_counter;
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
        memset(py_histo, 0, 16);
        tp.append_bytes(py_histo, 16);

        uint8_t roll_histo[16];
        memset(roll_histo, 0, 16);
        tp.append_bytes(roll_histo, 16);

        uint16_t sun_center_x[3], sun_center_y[3];
        memset(sun_center_x, 0, 3 * sizeof(uint16_t));
        memset(sun_center_y, 0, 3 * sizeof(uint16_t));
        for (int i = 0; i < 3; i++) {
            tp << (uint16_t)(sun_center_x[i] * 10) << (uint16_t)(sun_center_y[i] * 10);
        }

        uint16_t fiducial_x[4], fiducial_y[4];
        memset(fiducial_x, 0, 4 * sizeof(uint16_t));
        memset(fiducial_y, 0, 4 * sizeof(uint16_t));
        for (int i = 0; i < 4; i++) {
            tp << (uint16_t)(fiducial_x[i] * 10) << (uint16_t)(fiducial_y[i] * 10);
        }

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

        usleep(USLEEP_UDP_LISTEN);
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
        }
        printf("\n");

        delete packet;
    }

    printf("CommandListener thread #%ld exiting\n", tid);
    comReceiver.close_connection();
    started[tid] = false;
    pthread_exit( NULL );
}

void queue_cmd_proc_ack_tmpacket( uint64_t error_code )
{
    TelemetryPacket ack_tp(latest_system_id, TM_ACK, command_sequence_number, oeb_get_clock());
    ack_tp << error_code;
    tm_packet_queue << ack_tp;
}

void queue_settings_tmpacket()
{
    TelemetryPacket tp(SYS_ID_ASP, TM_SETTINGS, 0, oeb_get_clock());

    uint16_t last_parameter_table = 0;
    tp << last_parameter_table;

    uint8_t py_fps = 5;
    uint8_t py_gain = 1;
    uint16_t py_exposure = 1000;
    tp << py_fps << py_gain << py_exposure;

    uint8_t roll_fps = 5;
    uint8_t roll_gain = 1;
    uint16_t roll_exposure = 1000;
    tp << roll_fps << roll_gain << roll_exposure;

    tm_packet_queue << tp;
}

void *CommandHandlerThread(void *threadargs)
{
    // command error code definition
    // error_code   description
    // 0x0000       command implemented successfully
    // 0x0001       command not implemented
    // 0xEEEE       unknown command
    // 0xFFFF       unknown system ID
    // 
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    struct Thread_data *my_data;
    uint64_t error_code = 0x0001;
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
                    break;
                case 0xD2: //Set cadence of A2D temperatures packet
                    break;
                case 0xD3: //Set cadence of science packet
                    break;
                case 0xE0: //Load parameter table
                    break;
                case 0xF0: //Restart worker threads, handle elsewhere!
                    break;
                case 0xF1: //Restart all threads, handle elsewhere!
                    break;
                case 0xF2: //Restart runtime, handle elsewhere!
                    break;
                case 0xFF: //Graceful computer shutdown, handle elsewhere!
                    break;
                default:
                    std::cerr << "Unknown command\n";
                    error_code = 0xEEEE; //unknown command
            } //switch for command key
            break;
        case SYS_ID_PYC:
        case SYS_ID_RC:
            switch(my_data->command_key & 0xF)
            {
                case 0x4: //Set FPS
                    break;
                case 0x5: //Set gain
                    break;
                case 0x6: //Set exposure
                    break;
                case 0xC: //Send latest image
                    break;
                case 0xD: //Send specific image
                    break;
                default:
                    std::cerr << "Unknown command\n";
                    error_code = 0xEEEE; //unknown command
            } //switch for command key
            break;
        default:
            std::cerr << "Unknown system ID\n";
            error_code = 0xFFFF; //unknown system ID
    } //switch for system ID

    queue_cmd_proc_ack_tmpacket( error_code );

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
        usleep(USLEEP_IRS);
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

void *GRASPReceiverThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("GRASPReceiver thread #%ld!\n", tid);

    TelemetryReceiver telReceiver(44444);
    telReceiver.init_connection();

    while(!stop_message[tid])
    {
        unsigned int packet_length;

        usleep(USLEEP_UDP_LISTEN);
        packet_length = telReceiver.listen( );
        uint8_t *packet;
        packet = new uint8_t[packet_length];
        telReceiver.get_packet( packet );

        TelemetryPacket tp( packet, packet_length );

        switch(tp.getTmType())
        {
            case 0: //Pitch-yaw camera
                tp >> temp_py;
                break;
            case 1: //Roll camera
                tp >> temp_roll;
                break;
            default:
                break;
        }
    }

    printf("GRASPReceiver thread #%ld exiting\n", tid);
    telReceiver.close_connection();
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
            queue_cmd_proc_ack_tmpacket(0);
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
            std::cerr << "Graceful shutdown not yet implemented!\n";
            break;
        default:
            start_thread(CommandHandlerThread, &tdata);
    } //switch
}

void start_all_workers()
{
    start_thread(TelemetryHousekeepingThread, NULL);
    start_thread(TelemetryA2DThread, NULL);
    start_thread(TelemetryScienceThread, NULL);
    start_thread(TelemetrySenderThread, NULL);
    start_thread(IRSensorThread, NULL);
    start_thread(GRASPReceiverThread, NULL);
}

int main(void)
{  
    // to catch a Ctrl-C or termination signal and clean up
    signal(SIGINT, &sig_handler);
    signal(SIGTERM, &sig_handler);

    if(oeb_init() != 0) return 0;
    if(iopl(3) != 0) return 0;

    pthread_mutex_init(&mutexStartThread, NULL);

    /* Create worker threads */
    printf("In main: creating threads\n");

    for(int i = 0; i < MAX_THREADS; i++ ){
        started[0] = false;
    }

    // start the listen for commands thread right away
    start_thread(CommandListenerThread, NULL);
    start_all_workers();

    while(g_running){
        usleep(USLEEP_MAIN);

        // check if new command have been added to command queue and service them
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
    pthread_exit(NULL);

    return 0;
}
