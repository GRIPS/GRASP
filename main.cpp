#define MAX_THREADS 30
#define LOG_PACKETS false

#define SAVE_LOCATION "./"

//Sleep settings (seconds)
#define SLEEP_KILL             2 // waits when killing all threads

//Sleep settings (microseconds)
#define USLEEP_CMD_SEND     5000 // period for popping off the command queue
#define USLEEP_TM_SEND     50000 // period for popping off the telemetry queue
#define USLEEP_TM_GENERIC 950000 // period for adding generic telemetry packets to queue
#define USLEEP_UDP_LISTEN   1000 // safety measure in case UDP listening is changed to non-blocking
#define USLEEP_MAIN         5000 // period for checking for new commands

//IP addresses
#define IP_FC      "192.168.2.100"

#define IP_LOOPBACK "127.0.0.1"

//UDP ports, aside from PORT_IMAGE, which is TCP
#define PORT_CMD      50501 // commands, FC (receive)
#define PORT_TM       60501 // send telemetry to FC

//GRIPS system ID
#define SYS_ID_FC 0x00
#define SYS_ID_ASP 0x40

//GRIPS telemetry types
#define TM_ACK 0x01

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

// global declarations
uint8_t command_sequence_number = -1;
uint8_t latest_command_key = 0xFF;

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
    int camera_id;
    uint8_t command_key;
    uint8_t command_num_vars;
    uint16_t command_vars[15];
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
void *TelemetryPackagerThread(void *threadargs);

void *CommandListenerThread(void *threadargs);
void cmd_process_command(CommandPacket &cp);
void *CommandHandlerThread(void *threadargs);
void queue_cmd_proc_ack_tmpacket( uint16_t error_code );

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

    char timestamp[14];
    char filename[128];
    std::ofstream log;

/*
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

void *TelemetryPackagerThread(void *threadargs)
{
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    printf("TelemetryPackager thread #%ld!\n", tid);

    uint32_t tm_frame_sequence_number = 0;

    while(!stop_message[tid])
    {
        usleep(USLEEP_TM_GENERIC);
        tm_frame_sequence_number++;

        TelemetryPacket tp(SYS_ID_ASP, 0x00, tm_frame_sequence_number, 0x00000000);

        tm_packet_queue << tp;
    }

    printf("TelemetryPackager thread #%ld exiting\n", tid);
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

void queue_cmd_proc_ack_tmpacket( uint16_t error_code )
{
    TelemetryPacket ack_tp(SYS_ID_ASP, TM_ACK, command_sequence_number, 0x000000);
    ack_tp << error_code;
    tm_packet_queue << ack_tp;
}

void *CommandHandlerThread(void *threadargs)
{
    // command error code definition
    // error_code   description
    // 0x0000       command implemented successfully
    // 0x0001       command not implemented
    // 0xFFFF       unknown command
    // 
    long tid = (long)((struct Thread_data *)threadargs)->thread_id;
    struct Thread_data *my_data;
    uint16_t error_code = 0x0001;
    my_data = (struct Thread_data *) threadargs;

    queue_cmd_proc_ack_tmpacket( error_code );

    started[tid] = false;
    pthread_exit(NULL);
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

    uint16_t command = cp.getCmdType();

    Thread_data tdata;
    tdata.command_key = command;

    start_thread(CommandHandlerThread, &tdata);
}

void start_all_workers()
{
    start_thread(TelemetryPackagerThread, NULL);
    start_thread(TelemetrySenderThread, NULL);
}

int main(void)
{  
    // to catch a Ctrl-C or termination signal and clean up
    signal(SIGINT, &sig_handler);
    signal(SIGTERM, &sig_handler);

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

            latest_command_key = cp.getCmdType();

            printf("Received command key 0x%02X\n", latest_command_key);
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
