#include "output.h"
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <semaphore.h>
#include <string.h>

#include <libwebsockets.h>
#include "defines.h"
#include "utils.h"
#include "IPC/IPC.h"

extern bool servosEnabled;
extern uint8_t servoSpeed;
extern Servo rollStatusServo;
extern Servo pitchStatusServo;
extern SignalVector<attitudeT> attitudeSignal;
extern short desiredx;
extern short desiredy;
pthread_mutex_t messages_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t clients_mutex = PTHREAD_MUTEX_INITIALIZER;
sem_t sem;

unsigned int clients = 0;
unsigned int fds[MAX_CLIENTS];
struct libwebsocket * wsis[MAX_CLIENTS];
struct libwebsocket_context * contexts[MAX_CLIENTS];
std::string messages[MAX_CLIENTS][NUM_TYPES];

static int callback_http(struct libwebsocket_context *context,
                         struct libwebsocket *wsi,
                         enum libwebsocket_callback_reasons reason, void *user,
                         void *in, size_t len) 
{
    return 0;
}

static int callback_dumb_increment(struct libwebsocket_context * context,
                                   struct libwebsocket *wsi,
                                   enum libwebsocket_callback_reasons reason,
                                   void *user, void *in, size_t len)
{
  pthread_mutex_lock( &clients_mutex );
  switch (reason) {
    case LWS_CALLBACK_ESTABLISHED: { // just log message that someone is connecting      
      if (clients >= MAX_CLIENTS) {
        TRACE(WARNING, "Max clients limit reached.\n");
        pthread_mutex_unlock( &clients_mutex );
        return -1;
      }
      int fd = libwebsocket_get_socket_fd(wsi);
      int clientId;
      
      // search free slot for client
      clients++;
      for (clientId = 0; clientId < MAX_CLIENTS; clientId++) {
        if (fds[clientId] == 0) {
          fds[clientId] = fd;
          wsis[clientId] = wsi;
          contexts[clientId] = context;
          break;
        }
      }
      
      // Clear possible previous messages
      for (int i; i < NUM_TYPES; i++)
        messages[clientId][i].clear();
      
      TRACE(NOTICE, "Connection established, client id: %d, num clients: %d, fd: %d\n", clientId, clients, fd);
      break;
    }
      
    case LWS_CALLBACK_CLOSED: {
      // get client id and free slot
      int clientId;
      int fd = libwebsocket_get_socket_fd(wsi);
      
      // search client fd 
      bool found = false;
      for (clientId = 0; clientId < MAX_CLIENTS; clientId++) {
        if (fds[clientId] == fd) {
          fds[clientId] = 0;
          found = true;
          break;
        }
      }
      if (found) {
        clients--;
        TRACE(NOTICE, "Client %d disconnected, fd: %d.\n", clientId, fd);
      }
      else
        TRACE(NOTICE, "Unknown client disconnected, fd: %d.\n", fd);
      break;
    }
    
    case LWS_CALLBACK_SERVER_WRITEABLE: {
      // get client id and free slot
      int clientId;
      int fd = libwebsocket_get_socket_fd(wsi);
      
      // search client fd
      bool found = false;
      for (clientId = 0; clientId < MAX_CLIENTS; clientId++) {
        if (fds[clientId] == fd) {
          found = true;   
          break;
        }
      }
      
      if (found) {   
        for (int i = 0; i < NUM_TYPES; i++) {
            std::string message;
            message = messages[clientId][i];
            messages[clientId][i].clear();
            
            if (message != "") {
            // create a buffer to hold our response
            // it has to have some pre and post padding. You don't need to care
            // what comes there, libwebsockets will do everything for you. For more info see
            // http://git.warmcat.com/cgi-bin/cgit/libwebsockets/tree/lib/libwebsockets.h#n597
            unsigned char *buf = (unsigned char*) malloc(LWS_SEND_BUFFER_PRE_PADDING + message.size() + LWS_SEND_BUFFER_POST_PADDING);
        
            const char *c_message = message.c_str();
            for (int j=0; j < message.size(); j++)
                buf[LWS_SEND_BUFFER_PRE_PADDING + j] = c_message[j];
        
            // Send data
            libwebsocket_write(wsi, &buf[LWS_SEND_BUFFER_PRE_PADDING], message.size(), LWS_WRITE_TEXT);
            free(buf);
            }
        }
      }
      else {
          TRACE(WARNING, "WARNING: client id not found, fd: %d\n", fd);
      }
      
      // and schedule ourselves again
      //libwebsocket_callback_on_writable(context, wsi);
      break;
    }
    case LWS_CALLBACK_RECEIVE: // the funny part
      if (len > 0 && (*(char*)in) == 'e')
          servosEnabled = true;
      else if (len > 0 && (*(char*)in) == 'd')
          servosEnabled = false;
      else if (len > 0 && (*(char*)in) == 's') {
          servoSpeed = ((*((char*)in + 1)) - 48) * 10 + (*((char*)in + 2)) - 48;
          TRACE(NOTICE, "SERVO SPEED CHANGED: %d\n", servoSpeed);
      }
      else if (len > 0 && (*(char*)in) == 'm') {
          int speed = ((*((char*)in + 1) - 48) * 10 + *((char*)in + 2) - 48) * 10;
          
          rollStatusServo.setSpeed(speed);
          TRACE(NOTICE, "ROLL MAX SERVO SPEED CHANGED: %d\n", speed);
      }
      else if (len > 0 && (*(char*)in) == 'n') {
          int speed = ((*((char*)in + 1) - 48) * 10 + *((char*)in + 2) - 48) * 10;
          
          pitchStatusServo.setSpeed(speed);
          TRACE(NOTICE, "PITCH MAX SERVO SPEED CHANGED: %d\n", speed);
      }
      else if (len > 0 && (*(char*)in) == 'y') {
          long int value = ((*((char*)in + 1) - 48) * 10 + *((char*)in + 2) - 48) * 1000;
          rollStatusServo.setResponseDelay(value);
          
          TRACE(NOTICE, "ROLL RESPONSE DELAY CHANGED: %d\n", value);
      }
      else if (len > 0 && (*(char*)in) == 'z') {
          long int value = ((*((char*)in + 1) - 48) * 10 + *((char*)in + 2) - 48) * 1000;
          pitchStatusServo.setResponseDelay(value);
          
          TRACE(NOTICE, "PITCH RESPONSE DELAY CHANGED: %d\n", value);
      }
      else if (len > 0 && (*(char*)in) == 'v') {
          int value = ((*((char*)in + 1) - 48) * 10 + *((char*)in + 2) - 48) * 1000;
          rollStatusServo.setRange(value);
          
          TRACE(NOTICE, "ROLL RANGE CHANGED: %d\n", value);
      }
      else if (len > 0 && (*(char*)in) == 'w') {
          int value = ((*((char*)in + 1) - 48) * 10 + *((char*)in + 2) - 48) * 1000;
          pitchStatusServo.setRange(value);
          
          TRACE(NOTICE, "PITCH RANGE CHANGED: %d\n", value);
      }
      else if (len > 0 && (*(char*)in) == 'o') {
          int value = \
            + (*((char*)in + 1) - 48) * 100 \
            + (*((char*)in + 2) - 48) * 10 \
            +  *((char*)in + 2) - 547; // -48 - 499
          rollStatusServo.setOffset(value/10.f);
          
          TRACE(NOTICE, "ROLL OFFSET CHANGED: %d\n", value);
      }
      else if (len > 0 && (*(char*)in) == 'p') {
          int value = \
            + (*((char*)in + 1) - 48) * 100 \
            + (*((char*)in + 2) - 48) * 10 \
            +  *((char*)in + 2) - 547; // -48 - 499
          pitchStatusServo.setOffset(value/10.f);
          
          TRACE(NOTICE, "PITCH OFFSET CHANGED: %d\n", value);
      }
      else if (len > 0 && (*(char*)in) == 'a') {
          long int attitudeDelay = ((*((char*)in + 1) - 48) * 10 + *((char*)in + 2) - 48) * 1000;
          attitudeSignal.updateResponseDelay(attitudeDelay);
          
          TRACE(NOTICE, "ATTITUDE DELAY CHANGED: %d\n", attitudeDelay);
      }
      else if (len >= 7 && (*(char*)in) == 't') {
          int x = (*((char*)in + 2) - 48) * 100 + (*((char*)in + 3) - 48) * 10 + *((char*)in + 4) - 48;
          if ( (*((char*)in + 1)) == '-' )
              x = -x;
          desiredx = x;
          
          int y = (*((char*)in + 6) - 48) * 100 + (*((char*)in + 7) - 48) * 10 + *((char*)in + 8) - 48;
          if ( (*((char*)in + 5)) == '-' )
              y = -y;
          desiredy = y;
          
          TRACE(NOTICE, "POSITION CHANGED: (%d, %d)\n", x, y);
      }
      break;
      
    default:
      break;
  }
  
  pthread_mutex_unlock( &clients_mutex );
  return 0;
}

static struct libwebsocket_protocols protocols[] = {
    /* first protocol must always be HTTP handler */
    {
        "http-only",   // name
        callback_http, // callback
        0              // per_session_data_size
    },
    {
        "dumb-increment-protocol", // protocol name - very important!
        callback_dumb_increment,   // callback
        0                          // we don't use any per session data
    },
    { NULL, NULL, 0, 0 } /* terminator */
};

void mainThread(int port) {
  int use_ssl = 0;
  int opts = 0;
  char interface_name[128] = "";
  const char *iface = NULL;
          
  struct lws_context_creation_info info;
  struct libwebsocket_context *context;
  
  memset(&info, 0, sizeof info);
  info.port = port;
  info.iface = iface;
  info.protocols = protocols;
  info.extensions = libwebsocket_get_internal_extensions();
  info.ssl_cert_filepath = NULL;
  info.ssl_private_key_filepath = NULL;
  info.gid = -1;
  info.uid = -1;  
  info.options = opts;
  
  // create libwebsocket context representing this server
  context = libwebsocket_create_context(&info);
  
  if (context == NULL) {
      fprintf(stderr, "libwebsocket init failed\n");
      return;
  }
  
  TRACE(DEBUG, "starting server...\n");
  
  // infinite loop, to end this server send SIGTERM. (CTRL+C)
  while (1) {
      libwebsocket_service(context, 20);
      // libwebsocket_service will process all waiting events with their
      // callback functions and then wait 50 ms.
      // (this is a single threaded webserver and this will keep our server
      // from generating load while there are not requests to process)
  }
  
  libwebsocket_context_destroy(context);  
}

Output::Output(int port) {
  for (int i = 0; i < MAX_CLIENTS; i++)
    fds[i] = 0;
  
  
  sem_init(&sem, 0, 0);
  std::thread thread(mainThread, port); 
  thread.detach();
}

void Output::Send(std::string message, int type) {
  pthread_mutex_lock( &clients_mutex );
  for (int clientId = 0; clientId < MAX_CLIENTS; clientId++) {
    int fd = fds[clientId];
    struct libwebsocket * wsi = wsis[clientId];
    struct libwebsocket_context * context = contexts[clientId];
    
    // check if the client exists
    if (fd != 0) {
      messages[clientId][type] = message;
      
      //TRACE(DEBUG, "Calling libwebsocket_callback_on_writable to the client %d (fd: %d).\n", clientId, fd);
      libwebsocket_callback_on_writable(context, wsi);
    } 
    
  }
  pthread_mutex_unlock( &clients_mutex );
}