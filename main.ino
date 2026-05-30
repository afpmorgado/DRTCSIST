/*DEPENDENCIES

FreeRTOS is included in the rp2040 package installed in the Arduino IDE.
*/

#include <stdio.h>
#include <math.h>
#include "PID.h"
#include "circular_buffer.h"
#include <list>
#include <vector>
#include <stdexcept>
#include "mcp2515.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include <thread>
#include <string>
#include "pico/multicore.h"
#include <algorithm>
#include <ctime>
#include "hardware/pwm.h"

/*Some constants used in the code*/

#define R_10k 10000.0     // Fixed resistor
#define VREF 3.3          // Reference voltage
#define AnalogInputPin 26 // LDR input pin
#define LED_PIN 15        // LED PWM output
#define RES 12            // ADC resolution
#define DAC_RANGE 4096    // Max PWM output value
#define NUM_SAMPLES 30    // Number of readings to average (to filter noise)

using namespace std;

/* FreeRTOS -> handling shared resources between Tasks*/

SemaphoreHandle_t resourceMutex;
SemaphoreHandle_t resourceMutex2;

/* CAN-BUS Communication */
uint8_t node_address;
pico_unique_board_id_t pico_flash_id;

const int interruptPin{20};
volatile bool data_available = false;
MCP2515 can0{spi0, 17, 19, 16, 18, 10000000};

// Luminaire IDs -> Used for filtering messages
// and for sending messages to specific nodes

int my_id = 1;
int dest_id = 0;

struct can_frame canMsgTx, canMsgRx; // Not used

list<int> luminaire_ids;       // list to store the ids of the nodes in the network
list<uint8_t> luminaire_addrs; // not used

QueueHandle_t canMessageQueue;      // Queue for sending messages
QueueHandle_t receivedMessageQueue; // Queue for receiving messages

// For msg creation:
const int BUFSZ = 100;
char printbuf[BUFSZ];
const int BUFSZ_RCV = 8;
char rcv_buf[BUFSZ_RCV];
unsigned short int temp_id = 0; // temporary id to grant that the node that is waiting for the "real" id reads the proper messages from node 1

int is_waiting = 0;             // stage of the node during boot up
bool rcv_hb_while_wait = false; // flag, if node receives the heartbeat while waiting for the id -> not the first node in the network
bool rcv_cmd = false;           // flag that indicates that the node has received a command

int tgt_node = 0;        // node requested by the command in Serial
char asked_cmd[5];       // command asked by the user
bool awaiting_R = false; // if node is awaiting a response from a command request

// for real-time printing
int req_print_id = 0;
int print_RT = 0;

// For Task handling
TaskHandle_t handle1;
TaskHandle_t handle2;
TaskHandle_t handle3;
TaskHandle_t handle4;
TaskHandle_t handle5;

// For Heartbeat:
unsigned long t_beat = 0;
unsigned long run_time = 0;
unsigned long prev_run_time = 0;
vector<int> t_last_heartbeat;

// FOR ILUM:

float R_10LUX = 225e3; // Approximate range from 150K to 300K
float m = -0.9;
float b = 0.8 + log10(R_10LUX);

// FOR Calib:
bool calib_in_progress = false; // STOPS ALL LUX TYPE ACTIONS
bool I_started_calib = false;   // Marks the node that started calib
vector<float> K_vector;         // Gain vector
vector<bool> nodes_calib_ready; // Ready for calib nodes vector
float dark_offset = 0;

// Some characteristics of the physical system
const int SENSOR_PIN = A0;

// sampling interval
unsigned long previous_Time = 0;    // mili
unsigned long sample_Interval = 10; // mili

float Rx = 0;
float Vldr = 0;

// Used in Calibration
float gain = 0; // approximated linearized slope of the (volt (x),lux(y)) function
float H = 0;    // maps relation (reference_value_lux /  reference_value_volt)
float lux1 = 0;
float lux2 = 0;

int controller = 0;   // Is controller off or on
float reference = 20; // Reference value to drive system to
float ext_light = 0;  // exterior light detected AKA disturbance

float lux = 0;
float Duty_cycle = 0;
float N = 1;
float calc_b = 0;
float b_pid = 1;

int occupancy = 0;    // state of occupancy of the desk
const int LED_id = 1; // desk id

String input_Serial = ""; // stores the command input

// Auxiliar variable to determine what is to be streamed
int print_y = 0;
int print_u = 0;
int print_y_hub = 0;
int print_u_hub = 0;

// P measured in the led resistance when driven to max intensity.
const float Pmax = 1.63e-2;

// Used to compute and store the required errors, according to the formulas expressed in the project document
float visibility_error = 0;
float flicker_error = 0;
float aux_E = 0;
float Duty_cycle1 = 0;
float Duty_cycle2 = 0;

// Auxiliar variables to find average visibility error for a specific interval of time.
float N_aux = 0;
float visibility_error_aux = 0;

// PI initialization
pid PID_controller{0.18, 1, 0.1, 0.01};
// pid PID_controller {0.1, 1, 0.15, 0.01 };

// Circular buffer initialization for constant streaming of required values ("s" command)
CircularBuffer buffer1(6000);
CircularBuffer buffer2(6000);

// NODE STRUCT
//  FOR Distributed Control (CONSENSUS)
float c = 1;
float rho = 1.0;
bool do_consensus = true;
bool stage_one = true;
vector<float> u_v_1;
vector<bool> u_ready;

struct Node
{
  float c, L, d, n, m, cost;
  float u_av = 0.0;

  std::vector<float> k_row;
  std::vector<float> u_vector;
  std::vector<float> lambda;
};

Node initialize_node()
{
  Node node;
  node.c = c;
  node.L = reference;
  node.d = dark_offset;
  node.k_row = K_vector;
  node.u_av = 0.0;
  node.n = 0.0;
  node.cost = 0.0;
  node.m = 0.0;

  node.k_row = std::vector<float>(5, 0.0);    // One entry per node
  node.u_vector = std::vector<float>(5, 0.0); // One entry per node
  node.lambda = std::vector<float>(5, 0.0);
  return node;
}

Node add_ks_to_node(Node &node)
{
  node.c = c;
  node.L = reference;
  node.d = dark_offset;
  node.u_av = 0.0;
  node.n = 0.0;
  node.cost = 0.0;
  node.k_row.resize(K_vector.size());
  node.u_vector.resize(K_vector.size());
  node.lambda.resize(K_vector.size());

  for (size_t i = 0; i < K_vector.size(); ++i)
  {
    node.k_row[i] = K_vector[i];
  }

  node.m = node.n - pow(K_vector[my_id - 1], 2);

  for (size_t i = 0; i < K_vector.size(); ++i)
  {
    node.n += pow(K_vector[i], 2);
  }
  return node;
}

Node prep_node(Node &node)
{
  node.c = c;
  node.L = reference;
  node.d = dark_offset;
  node.u_av = 0.0;
  node.cost = 0.0;

  for (size_t i = 0; i < u_v_1.size(); ++i)
  {
    node.u_vector[i] = u_v_1[i];
  }
  return node;
}

// Global instance of Node
Node node1 = initialize_node();

////////////////////////////////////////////////////////////////////// END OF GLOBAL VARIABLES

///////////////////////////////////////////////////////////////////////////////// Luminaire Auxiliary Functions

float CountsToVolts(int Counts)
{
  /*  Args: Counts (0 to DAC_RANGE) (PWM value)
      Returns: Voltage (Volts)

      Objective: By using the DAC_RANGE and VREF convert PWM value into volts
  */
  return Counts * (VREF / DAC_RANGE);
}

float LDR_Resistance(float Vin)
{
  /*
      Args: Vin (Voltage across the LDR)

      Returns: Resistance of the LDR in ohms

      Objective: Use the voltage divider formula to compute the resistance
                 of the LDR given the input voltage (Vin). Assumes a 10kΩ
                 resistor in series with the LDR.
  */
  if (Vin == 0)
    return 0;
  return R_10k * ((VREF / Vin) - 1);
}

float voltageToLux(float LDR)
{
  /*
      Args: LDR (Resistance of the LDR in ohms)

      Returns: Light intensity in Lux

      Objective: Use the logarithmic sensor model to convert resistance
                 to Lux based on calibration parameters (m, b).
  */
  return pow(10, ((log10(LDR) - b) / m));
}

float LuxToVoltage(float LUX)
{
  /*
      Args: LUX (Light intensity in Lux)

      Returns: Expected voltage across the LDR (Vin)

      Objective: Inverse of voltageToLux. Convert Lux to LDR resistance,
                 then use voltage divider to find the voltage across the LDR.
  */
  float LDR = pow(10, ((log10(LUX) * m) + b));
  return VREF / ((LDR / R_10k) + 1);
}

float GetLUX()
{
  /*
      Returns: Light intensity in Lux

      Objective: Sample analog input multiple times, average the readings,
                 convert to voltage, compute LDR resistance, and finally
                 estimate light intensity in Lux.
  */
  float total_Vin = 0;

  // Take multiple readings to reduce noise
  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    total_Vin += CountsToVolts(analogRead(AnalogInputPin));
  }

  // Get the average voltage
  float Vin = total_Vin / NUM_SAMPLES;

  // Convert to LDR resistance and then to Lux
  float LDR = LDR_Resistance(Vin);
  return voltageToLux(LDR);
}

///////////////////////////////////////////////////////////////////////////////// IDS Auxiliary Functions

void printIDList()
{
  /*
      Objective: Print the current list of network node IDs (luminaire_ids).
  */

  String output = "Network Nodes: ";
  for (int id : luminaire_ids)
  {
    output += String(id) + " "; // Append each ID followed by a space
  }
  Serial.println(output); // Print the entire list in one line
}

void sendIDList(unsigned short identifier)
{
  /*
      Args: identifier (unique CAN identifier for the message recipient)

      Objective: Send the sorted list of luminaire node IDs over
                 CAN via a message queue (used when adding a node to the network (which isn't the first in the network). Sends a final "DONE" message
                 to signal the end of transmission. Communicates the existant nodes in the network to a new one.
                 Is sent by node with id = 1 (any node can be id = 1).
  */
  if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
  { // Ensure thread-safe access to the list
    luminaire_ids.sort();
    for (int id : luminaire_ids)
    {
      char idMessage[8] = {0};                                           // Limit to 8 bytes
      snprintf(idMessage, sizeof(idMessage), "h%d %hu", id, identifier); // Format the message as 'h <number>'
      xQueueSend(canMessageQueue, &idMessage, portMAX_DELAY);            // Send the message to the queue                            //wait
    }

    // Send a "DONE" message to indicate the end of the list
    char doneMessage[8] = {0};                                       // Format the "DONE" message
    snprintf(doneMessage, sizeof(doneMessage), "D %hu", identifier); // Format the message as 'h <number>'
    xQueueSend(canMessageQueue, &doneMessage, portMAX_DELAY);        // Send the "DONE" message to the queue                          //wait

    xSemaphoreGive(resourceMutex); // Release the mutex
  }
}

int findFirstFreeID()
{
  /*
      Returns: The smallest available integer ID not already assigned

      Objective: Iterate through sorted luminaire_ids to find the first
                 unused ID (excluding my_id). So there is only one of each.
  */
  int id = 1;
  luminaire_ids.sort();
  // Ensure we don’t assign an ID already taken, including my_id
  while (true)
  {
    bool id_taken = false;
    for (int existing_id : luminaire_ids)
    {
      if (existing_id == id)
      {
        id_taken = true;
        break;
      }
    }
    if (!id_taken && id != my_id)
    { // Don’t assign my own ID
      return id;
    }
    id++;
  }
}

int id_off_lst()
{
  /*
      Returns: The smallest available integer ID not already assigned.

      Objective: If list (there is no other node) is empty puts your node at id = 1, then
                 Iterate through sorted luminaire_ids to find the first
                 unused ID (excluding my_id) (by calling findFirstFreeID).
  */
  if (luminaire_ids.empty())
  {
    luminaire_ids.push_back(my_id);
    return 1;
  }

  return findFirstFreeID();
}

bool id_exists(int id_to_search)
{
  /*
      Args: id_to_search (ID to check in the list)

      Returns: true if the ID exists, false otherwise

      Objective: Determine whether a ID exists (Done by checking the list).
                 Sorts the list for consistency before checking.
  */
  if (luminaire_ids.empty())
  {
    return false;
  }

  luminaire_ids.sort();

  for (int id : luminaire_ids)
  {
    if (id == id_to_search)
    {
      return true;
    }
  }

  return false;
}

///////////////////////////////////////////////////////////////////////////////// Process, receive and send CAN messages

void canReceiveTask(void *pvParameters)
{
  /*
      Note: this function running in a separate task

      Objective: Check the CAN controller for incoming messages.
                 If a message is received, push it onto the receivedMessageQueue.
  */
  struct can_frame frm0;

  while (true)
  {
    uint8_t irq = can0.getInterrupts();
    if (irq & MCP2515::CANINTF_RX0IF)
    {
      if (can0.readMessage(&frm0) == MCP2515::ERROR_OK)
      {
        xQueueSend(receivedMessageQueue, &frm0, portMAX_DELAY);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void processReceivedMessagesTask(void *pvParameters)
{
  /*
      Note: this function running in a separate task

      Objective: Continuously process messages received via CAN.
                 Handles ID management, control updates, and calibration messaging.
  */
  struct can_frame receivedFrame;
  while (true)
  {
    if (xQueueReceive(receivedMessageQueue, &receivedFrame, portMAX_DELAY) == pdTRUE)
    {
      // Extract message
      char message[9] = {0}; // 8 bytes + null terminator
      memcpy(message, receivedFrame.data, receivedFrame.can_dlc);
      message[receivedFrame.can_dlc] = '\0'; // Null-terminate at the correct length

      Serial.printf("Message Received: %s\n", message);

      // To do calibration
      process_calib(message); // FOR CALIB

      long long val = 0;

      // For real-time printing command
      if (strncmp(message, "LU", 2) == 0)
      {
        int meas_lux = 0;
        if (sscanf(message, "LU%d", &meas_lux) == 2)
        { // Parse the integer
          if (print_RT == 1)
          {
            print_real_time(meas_lux, 0);
          }
        }
      }
      if (strncmp(message, "DC", 2) == 0)
      {
        int meas_dc = 0;
        if (sscanf(message, "DC%d", &meas_dc) == 2)
        { // Parse the integer
          if (print_RT == 1)
          {
            print_real_time(meas_dc, 1);
          }
        }
      }
      // To get consensus message
      if (sscanf(message, "%llu", &val) == 1)
      {
        int index = (int)(val / pow(10, 10)); // Recover the original ID
        if (index > 0)
        {
          long long tmpy = (long long)(index * pow(10, 10));
          float u_ot = (float)((val / pow(10, 9)) - tmpy); // Convert the scaled float back to the original value
          // int id_temporary = (int)index;
          Serial.printf("Parsed index = %d, value = %f\n", index, u_ot);
          node1.u_vector[index - 1] = u_ot;
          u_ready[index - 1] = true;
          String output = "K Values: ";
          for (int i = 0; i < u_ready.size(); i++)
          {
            output += String(u_ready[i]) + " "; // Assuming each element has an 'id' field
          };
          Serial.println(output); // Print the entire list in one line

          return;
        }
      }

      // To get heartbeat message from other live nodes
      if (strncmp(message, "LV", 2) == 0)
      {
        process_heart_beat(message);
      }

      // To print the id of the node and the final list of ids
      if (strncmp(message, "D", 1) == 0)
      { // Compare up to 4 characters
        unsigned short identifier = 0;
        if (sscanf(message, "D %hu", &identifier) == 1)
        { // Parse the integer

          if (identifier == temp_id)
          {
            printIDList();
            Serial.printf("My ID: %d\n", my_id);
          }
        }
      }

      // To acquire (receive) the nodes of the network sent by node 1
      if (strncmp(message, "h", 1) == 0)
      { // Check if the message starts with 'h '
        int extractedValue = 0;
        unsigned short identifier = 0;

        if (sscanf(message, "h%d %hu", &extractedValue, &identifier) == 2)
        { // Parse the integer
          if (identifier == temp_id)
          {
            if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
            {
              // Check if the value is already in the list
              if (!id_exists(extractedValue))
              {
                luminaire_ids.push_back(extractedValue); // Add the value if not present
              }
              luminaire_ids.sort(); // Ensure the list is ordered
              xSemaphoreGive(resourceMutex);
            }
          }
        }
        else
        {
          Serial.println("Failed to parse the integer from the message.");
        }
      }

      // a request received by node one to send the list of existant ids and attribute a new id
      if (strncmp(message, "R", 1) == 0 && my_id == 1 && is_waiting == 2)
      {
        int new_id;
        if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
        {
          luminaire_ids.sort();
          new_id = id_off_lst();
          luminaire_ids.push_back(new_id);
          xSemaphoreGive(resourceMutex);
        }
        char responseMessage[8] = {0};
        unsigned short identifier = 0;
        if (sscanf(message, "R%hu", &identifier) == 1)
        {
          snprintf(responseMessage, sizeof(responseMessage), "I%d %hu", new_id, identifier);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          sendIDList(identifier);
        }

        // Associated an id to the requester (received id)
      }
      else if (strncmp(message, "I", 1) == 0)
      {
        unsigned short identifier = 0;
        int assigned_id = 0;
        if (sscanf(message, "I%d %hu", &assigned_id, &identifier) == 2)
        { // Parse the integer

          if (identifier == temp_id)
          {
            if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
            {
              my_id = assigned_id;
              xSemaphoreGive(resourceMutex);
            }
          }
        }
      }
      else if (strncmp(message, "IDS:", 4) == 0)
      {
        luminaire_ids.clear();
        char *token = strtok(&message[4], ",");
        while (token != NULL)
        {
          luminaire_ids.push_back(atoi(token));
          token = strtok(NULL, ",");
        }
      }
      // Handling user commands between nodes
      if (awaiting_R)
      {
        if (strncmp(message, "Ai", 2) == 0)
        {
          int rcv_value;
          sscanf(message, "Ai%d", &rcv_value);
          if (rcv_value == -1)
          {
            Serial.println("ack");
          }
          else
          {
            print_rcv_cmd(rcv_value, 0, 0);
          }
        }
        else if (strncmp(message, "Af", 2) == 0)
        {
          float rcv_value;
          sscanf(message, "Af%f", &rcv_value);
          if (rcv_value == -1)
          {
            Serial.println("ack");
          }
          else
          {
            print_rcv_cmd(0, rcv_value, 1);
          }
        }
      }
      else if (strncmp(message, "C", 1) == 0)
      {
        sscanf(message, "C%d", &dest_id);
        if (dest_id == my_id)
        {
          rcv_cmd = true;
        }
      }
      else if (message[0] == 'g')
      {
        char responseMessage[8] = {0};
        if (rcv_cmd == true)
        {
          rcv_cmd = false;

          if (message[1] == 'u')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", Duty_cycle);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'g')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", gain);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'r')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", reference);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'y')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", lux);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'v')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", Vldr);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'o')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Ai%d", occupancy);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'a')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Ai%d", PID_controller.getAntiWindup());
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'f')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Ai%d", PID_controller.getFeedback());
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'd')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", ext_light);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'p')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", Pmax * Duty_cycle);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 't')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", float(micros()) / 1e6);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'E')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", Pmax * (aux_E / N));
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'V')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", visibility_error / N);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'Q')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", visibility_error_aux / N_aux);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'F')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Af%f", flicker_error / N);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
          if (message[1] == 'c')
          {
            snprintf(responseMessage, sizeof(responseMessage), "Ai%d", controller);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
        }
      }
      else if (message[0] == 'u')
      {
        if (rcv_cmd == true)
        {
          rcv_cmd = false;
          char responseMessage[8] = {0};
          float rcv_value = 0;
          sscanf(message, "u%f", &rcv_value);
          Duty_cycle = rcv_value;
          snprintf(responseMessage, sizeof(responseMessage), "Ai%d", -1);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
      }
      else if (message[0] == 'a')
      {
        if (rcv_cmd == true)
        {
          rcv_cmd = false;
          char responseMessage[8] = {0};
          int rcv_value = 0;
          sscanf(message, "a%d", &rcv_value);
          PID_controller.setAntiWindup(rcv_value);
          snprintf(responseMessage, sizeof(responseMessage), "Ai%d", -1);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
      }
      else if (message[0] == 'o')
      {
        if (rcv_cmd == true)
        {
          rcv_cmd = false;
          char responseMessage[8] = {0};
          int rcv_value = 0;
          sscanf(message, "o%d", &rcv_value);
          occupancy = rcv_value;
          if (occupancy == 0)
          {
            reference = 2;
          }
          else if (occupancy == 1)
          {
            reference = 8;
          }
          snprintf(responseMessage, sizeof(responseMessage), "Ai%d", -1);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
      }
      else if (message[0] == 'f')
      {
        if (rcv_cmd == true)
        {
          rcv_cmd = false;
          char responseMessage[8] = {0};
          int rcv_value = 0;
          sscanf(message, "f%d", &rcv_value);
          PID_controller.setFeedback(rcv_value);
          snprintf(responseMessage, sizeof(responseMessage), "Ai%d", -1);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
      }
      else if (message[0] == 'r')
      {
        if (rcv_cmd == true)
        {
          rcv_cmd = false;
          char responseMessage[8] = {0};
          int rcv_value = 0;
          sscanf(message, "r%f", &rcv_value);
          reference = rcv_value;
          snprintf(responseMessage, sizeof(responseMessage), "Ai%d", -1);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
      }
      else if (message[0] == 'c')
      {
        if (rcv_cmd == true)
        {
          rcv_cmd = false;
          char responseMessage[8] = {0};
          int rcv_value = 0;
          sscanf(message, "c%d", &rcv_value);
          controller = rcv_value;
          snprintf(responseMessage, sizeof(responseMessage), "Ai%d", -1);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
      }
      else if (message[0] == 's')
      {
        char responseMessage[8] = {0};
        char rcv_var;
        if (rcv_cmd == true)
        {
          rcv_cmd = false;
          sscanf(message, "S%c", &rcv_var);
          if (rcv_var == 'y')
          {
            print_y_hub = 1;
          }
          else if (rcv_var == 'u')
          {
            print_u_hub = 1;
            snprintf(responseMessage, sizeof(responseMessage), "Ai%d", -1);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
        }
      }
      else if (message[0] == 'S')
      {
        char responseMessage[8] = {0};
        char rcv_var;
        if (rcv_cmd == true)
        {
          rcv_cmd = false;
          sscanf(message, "S%c", &rcv_var);
          if (rcv_var == 'y')
          {
            print_y_hub = 0;
          }
          else if (rcv_var == 'u')
          {
            print_u_hub = 0;
            snprintf(responseMessage, sizeof(responseMessage), "Ai%d", -1);
            xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          }
        }
      }
    }
  }
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

void canSendTask(void *pvParameters)
{
  /*
      Note: This runs on a separate task.

      Args:
        pvParameters - FreeRTOS task parameter.

      Objective:
        Wait for messages from the `canMessageQueue` and transmit them
        via CAN. Message length is clamped to 8 bytes. The task yields
        periodically to allow scheduler fairness. Runs indefinitely as
        a FreeRTOS task.
  */
  while (true)
  {
    char messageToSend[8] = {0};
    if (xQueueReceive(canMessageQueue, messageToSend, portMAX_DELAY))
    {
      Serial.printf("Message Sent: %s\n", messageToSend);
      struct can_frame frm0;

      frm0.can_dlc = 8;                    // CAN messages typically use a fixed length up to 8 bytes
      memcpy(frm0.data, messageToSend, 8); // Copy exactly 8 bytes
      frm0.can_dlc = strlen(messageToSend) > 8 ? 8 : strlen(messageToSend);
      can0.sendMessage(&frm0);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

std::list<int>::iterator id_finder(int id_to_find)
{
  /*
      Args:
        id_to_find - ID to find in the luminaire ID list.

      Returns:
        Iterator pointing to the found ID, or luminaire_ids.end() if not found

      Objective:
        Efficiently locate an ID in the list.
  */

  auto it = std::find(luminaire_ids.begin(), luminaire_ids.end(), id_to_find);
  return (it != luminaire_ids.end()) ? it : luminaire_ids.end(); // Return iterator to found element or end()
}

//////////////////////////////////////////////////////////////////////////////////////////////////////// HEARTBEAT

void check_heart_beat()
{
  /*
      Objective:
        - Monitors heartbeats of all nodes.
        - Detects failures when a node hasn’t sent a heartbeat within 100ms.
        - If a dead node is detected:
            - Removes it from the list.
            - Adjusts IDs of higher-ranked nodes.
            - Ensures all tracking structures (like timestamps) are updated accordingly.
  */
  unsigned long current_time = micros();

  if (calib_in_progress)
  {
    for (unsigned i = 1; i < t_last_heartbeat.size() + 1; i++)
    {
      t_last_heartbeat.at(i - 1) = current_time;
      return;
    }
  }

  // Update my own timestamp first
  if (my_id <= t_last_heartbeat.size())
  {
    t_last_heartbeat[my_id - 1] = current_time;
  }

  for (unsigned i = 1; i < t_last_heartbeat.size() + 1; i++)
  {
    if (current_time - t_last_heartbeat.at(i - 1) >= 100050000)
    { // 100ms timeout
      if (i != my_id)
      {
        auto it = id_finder(i);
        if (it != luminaire_ids.end())
        {
          Serial.printf("Node %d is dead. Removing from list.\n", i);
          if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
          {
            luminaire_ids.erase(it);
            luminaire_ids.sort();
            if (my_id > i)
            {
              // Transition
              // Shift Left
              unsigned old_id = my_id;
              auto id = id_finder(my_id);
              luminaire_ids.erase(id);
              my_id = my_id - 1;
              luminaire_ids.push_back(my_id);
              u_ready[my_id - 1] = true;
              luminaire_ids.sort();
              if (xSemaphoreTake(resourceMutex2, portMAX_DELAY) == pdTRUE)
              {
                // Resize t_last_heartbeat to match active nodes
                if (t_last_heartbeat.size() > luminaire_ids.size())
                {
                  t_last_heartbeat.resize(luminaire_ids.size());
                }
                t_last_heartbeat[my_id - 1] = current_time; // My new slot as node 1
                if (old_id != 1 && old_id - 1 < t_last_heartbeat.size())
                {
                  t_last_heartbeat[old_id - 1] = 0; // Clear old slot
                }
                xSemaphoreGive(resourceMutex2);
              }
            }
            else
            {
              if (xSemaphoreTake(resourceMutex2, portMAX_DELAY) == pdTRUE)
              {
                t_last_heartbeat[i - 1] = 0;
                xSemaphoreGive(resourceMutex2);
              }
            }
            xSemaphoreGive(resourceMutex);
            break;
          }
        }
      }
    }
  }
}

void send_heart_beat(unsigned long t)
{
  /*
      Args:
        t - Current time in microseconds

      Objective:
        Periodically sends a heartbeat message (every 25ms) to signal this node is alive.
        Also updates this node's timestamp in the heartbeat tracker.
        Used to decide which node is first to join the network (hasn't received a heartbeat from any other node during bootup).
  */
  if (is_waiting == 2)
  {
    if (t - t_beat >= 25000000)
    { // Every 25ms
      snprintf(printbuf, BUFSZ, "LV %d\n", my_id);
      xQueueSend(canMessageQueue, &printbuf, portMAX_DELAY);
      t_beat = t;
      // Update my timestamp with current time
      if (xSemaphoreTake(resourceMutex2, portMAX_DELAY) == pdTRUE)
      {
        if (my_id <= t_last_heartbeat.size())
        {
          t_last_heartbeat[my_id - 1] = micros(); // Consistent with check_heart_beat
        }
        else
        {
          // Expand if needed (shouldn’t happen often)
          t_last_heartbeat.resize(my_id, 0);
          t_last_heartbeat[my_id - 1] = micros();
        }
        xSemaphoreGive(resourceMutex2);
      }
    }
  }
}

void process_heart_beat(const char *str)
{
  /*
      Args:
        str - Incoming CAN message.

      Returns:
        None

      Objective:
        Checks if the message is a heartbeat message.
        Parses heartbeat messages from other nodes.
        - If valid, updates the timestamp for that node.
        - If the node is new, adds it to the system and resizes heartbeat tracking.
        - Prevents modification if calibration is active or ID conflict detected.
  */
  if (calib_in_progress)
  {
    return;
  }
  int received_id = -1;
  if (sscanf(str, "LV %d", &received_id) == 1)
  {
    if (is_waiting == 1)
    {
      rcv_hb_while_wait = true;
      return;
    }

    if (is_waiting == 2)
    {

      if (received_id == my_id)
      {
        // Should not receive his own heartbeat message,
        // if it does Boot up was not done the right way
        return;
      }
      /* Checks if there is a position in list ids for id */
      if (id_exists(received_id))
      {
        /* sees if there is a position in vector t_heart_beats with id */
        if (t_last_heartbeat.size() < received_id)
        {
          if (xSemaphoreTake(resourceMutex2, portMAX_DELAY) == pdTRUE)
          {
            t_last_heartbeat.resize(received_id);
            xSemaphoreGive(resourceMutex2);
          }
        }
        if (xSemaphoreTake(resourceMutex2, portMAX_DELAY) == pdTRUE)
        {
          t_last_heartbeat.at(received_id - 1) = micros();
          xSemaphoreGive(resourceMutex2);
        }
      }
      else
      {
        // New node detected — add to system
        Serial.println("Adding node...\n");
        luminaire_ids.push_back(received_id);
        if (t_last_heartbeat.size() < received_id)
        {
          if (xSemaphoreTake(resourceMutex2, portMAX_DELAY) == pdTRUE)
          {
            t_last_heartbeat.resize(received_id);
            xSemaphoreGive(resourceMutex2);
          }
        }
        if (xSemaphoreTake(resourceMutex2, portMAX_DELAY) == pdTRUE)
        {
          t_last_heartbeat.at(received_id - 1) = micros();
          xSemaphoreGive(resourceMutex2);
        }
      }
    }
  }
}

void heart_beat(void *pvParameters)
{
  /*
      Note: this function runs on a separate task.

      Args:
        pvParameters - FreeRTOS task parameter

      Objective:
        Periodically performs heartbeat-related tasks:
        - Sends this node’s heartbeat
        - Checks other nodes' heartbeat status
        - Keeps luminaire ID list sorted
        Runs forever as a FreeRTOS task.
  */
  while (true)
  {
    vTaskDelay(10 / portTICK_PERIOD_MS);
    run_time = micros();

    send_heart_beat(run_time); // Send Heart Beat
    check_heart_beat();

    if (xSemaphoreTake(resourceMutex, portMAX_DELAY) == pdTRUE)
    {
      luminaire_ids.sort();
      xSemaphoreGive(resourceMutex);
    }

    prev_run_time = run_time;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////// CALIB

// For Calib
void calib_req()
{
  /*
      Objective:
        Begins the calibration process:
        - Prepares for calibration
        - Marks this node as ready
        - Sends calibration request message to other nodes
  */
  // User Asked for Calib
  prep_calib();
  Serial.print(luminaire_ids.size());

  // Init nodes_calib_ready / Reset nodes_calib_ready
  nodes_calib_ready.resize(luminaire_ids.size());
  std::fill(nodes_calib_ready.begin(), nodes_calib_ready.end(), false);

  nodes_calib_ready[my_id - 1] = true; // Node is ready (my_id - 1 vector starts at 0)
  I_started_calib = true;

  Serial.print("Suspending all control actions\n Starting Calibration\n");

  // Send calib request
  snprintf(rcv_buf, BUFSZ_RCV, "CL%d", my_id);
  xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
  calib(); // Begin calibration routine
}

void process_calib(const char *str)
{
  /*
      Args:
        str - Incoming CAN message

      Objective:
        Processes all messages related to calibration including:
        - Calibration done
        - Requests to enter calibration
        - Ready/Not ready responses
        - Light ON commands
        - Lux measurement triggers
  */
  int received_id = -1;

  // All nodes have finished calibrating
  if (strncmp(str, "CDONE", 5) == 0)
  {

    node1 = add_ks_to_node(node1);
    u_ready.resize(K_vector.size());
    std::fill(u_ready.begin(), u_ready.end(), false);
    u_ready[my_id - 1] = true;

    calib_in_progress = false;
    controller = 1;
    return;
  }
  // Calibration req msg
  if (strncmp(str, "CL", 2) == 0)
  {
    if (is_waiting == 2)
    {
      // Set to Dark
      prep_calib();
      // Send ready
      snprintf(rcv_buf, BUFSZ_RCV, "DK %d\n", my_id);
      xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
      return;
    }
    // Send not ready
    snprintf(rcv_buf, BUFSZ_RCV, "NRDYDK");
    xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
    return;
  }

  // Not ready -> Calib req response msg
  if (strncmp(str, "NRDYDK", 6) == 0)
  {
    calib_in_progress = false;
    Serial.println("Ending Calibration\n Warning: Calibration not complete  (Node not ready)\n");
    return;
  }

  // Dark -> Calib req response msg
  if (strncmp(str, "DK", 2) == 0)
  {

    if (sscanf(str, "DK %d", &received_id) == 1)
    {
      if (id_exists(received_id))
      {
        nodes_calib_ready[received_id - 1] = true; // Node is ready (received_id - 1 vector starts at 0)
        // Print the current state of nodes_calib_ready
        print_nodes_calib_ready_state();
        if (I_started_calib)
        {
          calib();
        }
        return;
      }
      else
      {
        // Send not ready
        snprintf(rcv_buf, BUFSZ_RCV, "NRDYDK");
        xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
        calib_in_progress = false;
        Serial.println("Ending Calibration\n Warning: Calibration not complete (Calib node does not exist)\n");
      }
    }
    else
    {
      // Send not ready
      snprintf(rcv_buf, BUFSZ_RCV, "NRDYDK");
      xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
      calib_in_progress = false;
      Serial.println("Ending Calibration\n Warning: Calibration not complete (Calib node does not exist)\n");
    }
  }

  // Dark -> Measure Dark
  if (strncmp(str, "DRKMR", 5) == 0)
  {
    // Read
    dark_offset = GetLUX(); // Dark offset
  }

  // White ON MEASURE
  if (strncmp(str, "WM", 2) == 0)
  {

    if (sscanf(str, "WM %d", &received_id) == 1)
    {
      if (id_exists(received_id))
      {
        // UPDATE K_VECTOR
        float measuredLux = GetLUX();
        Serial.printf("LUX - read - other: %f\n", measuredLux);
        K_vector[received_id - 1] = (measuredLux - dark_offset) / (1 - 0);
        print_calib();
        return;
      }
      else
      {
        // Send not ready
        snprintf(rcv_buf, BUFSZ_RCV, "NRDYDK");
        xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
        calib_in_progress = false;
        Serial.println("Ending Calibration\n Warning: Calibration not complete (Calib node does not exist)\n");
      }
    }
    else
    {
      // Send not ready
      snprintf(rcv_buf, BUFSZ_RCV, "NRDYDK");
      xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
      calib_in_progress = false;
      Serial.println("Ending Calibration\n Warning: Calibration not complete (Calib node does not exist)\n");
    }
  }

  // White ON ORDER -> Calib begins
  if (strncmp(str, "WH", 2) == 0)
  {

    if (sscanf(str, "WH %d", &received_id) == 1)
    {
      if (id_exists(received_id))
      {
        if (received_id == my_id)
        {
          // LIGHT ON ORDER RECEIVED
          Calib_Light_ON();
        }
        return;
      }
      else
      {
        // Send not ready
        snprintf(rcv_buf, BUFSZ_RCV, "NRDYDK");
        xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
        calib_in_progress = false;
        controller = 0;
        Serial.println("Ending Calibration\n Warning: Calibration not complete (Calib node does not exist)\n");
      }
    }
    else
    {
      // Send not ready
      snprintf(rcv_buf, BUFSZ_RCV, "NRDYDK");
      xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
      calib_in_progress = false;
      controller = 0;
      Serial.println("Ending Calibration\n Warning: Calibration not complete (Calib node does not exist)\n");
    }
  }
  return;
}

void print_nodes_calib_ready_state()
{
  /*
      Objective:
        Prints a summary of which nodes are currently marked as calibration-ready.
        Useful for debug/inspection.
  */

  String output = "Calib Ready State of nodes: ";
  for (int i = 0; i < nodes_calib_ready.size(); i++)
  {
    output += String(nodes_calib_ready[i]) + " "; // Assuming each element has an 'id' field
  }
  Serial.println(output); // Print the entire list in one line
}

void prep_calib()
{
  /*
      Objective:
        Resets the calibration state:
        - Clears K_vector and dark offset
        - Flags that calibration is in progress
        - Turns off the light for dark measurement
  */

  // Init K_vector / Reset K_vector
  K_vector.resize(luminaire_ids.size());
  std::fill(K_vector.begin(), K_vector.end(), 0);

  // Reset dark_offset
  dark_offset = 0;

  // Stops all lux actions not calib
  calib_in_progress = true;

  // Sets to Dark
  analogWrite(LED_PIN, 0);
}

void Calib_Light_ON()
{
  /*
      Objective:
        Performs self-calibration step:
        - light ON
        - Waits for stabilization
        - Sends measurement message to others
          (message that tells the others to measure)
        - Measures own LUX
        - Updates self entry in K_vector
        - light OFF
  */
  // Light ON
  analogWrite(LED_PIN, DAC_RANGE);

  // Allow stabilization
  vTaskDelay(4000 / portTICK_PERIOD_MS);

  // Sending light on msg
  snprintf(rcv_buf, BUFSZ_RCV, "WM %d", my_id);
  xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);

  float measuredLux = GetLUX();
  Serial.printf("LUX - read - self: %f\n", measuredLux);
  K_vector[my_id - 1] = (measuredLux - dark_offset) / (1 - 0);

  // Make sure everyone read
  vTaskDelay(4000 / portTICK_PERIOD_MS);

  analogWrite(LED_PIN, 0);

  print_calib();
}

void calib()
{
  /*
      Objective:
        Executes the full multi-node calibration sequence:
        - Waits until all nodes report 'ready'
        - Measures dark offset
        - Loops through all nodes to perform light ON calibration
        - Collects all gain values (K_vector)
  */
  // if all nodes ready
  if (std::find(nodes_calib_ready.begin(), nodes_calib_ready.end(), false) == nodes_calib_ready.end())
  {
    // Did not find false -> All nodes true
    // All nodes are Dark
    print_nodes_calib_ready_state();

    // Starting phase 2
    Serial.print("Starting Calibration phase 2\n");

    // Light OFF
    // Already is OFF but to make sure:
    analogWrite(LED_PIN, 0);

    // Sending dark measure msg
    snprintf(rcv_buf, BUFSZ_RCV, "DRKMR\n");
    xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);

    // Read
    dark_offset = GetLUX(); // Dark offset

    // Make sure everyone read
    vTaskDelay(4000 / portTICK_PERIOD_MS);

    luminaire_ids.sort(); // Make sure they are sorted

    // Loop for lights on
    for (int id : luminaire_ids)
    {
      // Sending light on order msg
      if (id != my_id)
      {
        snprintf(rcv_buf, BUFSZ_RCV, "WH %d", id);
        xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
      }

      if (id == my_id)
      {
        Calib_Light_ON();
      }
      else
      {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
      }
    }

    print_calib();

    // Sending Calib Done order msg
    snprintf(rcv_buf, BUFSZ_RCV, "CDONE");
    xQueueSend(canMessageQueue, &rcv_buf, portMAX_DELAY);
    calib_in_progress = false;
    node1 = add_ks_to_node(node1);

    u_ready.resize(K_vector.size());
    std::fill(u_ready.begin(), u_ready.end(), false);
    u_ready[my_id - 1] = true;

    gain = K_vector[my_id - 1];
    controller = 1;
    I_started_calib = false;
  }
}

void print_calib()
{
  /*
      Objective: Print the current calibration values (K_vector) and the dark offset.

      Returns: None
  */
  String output = "K Values: ";
  for (int i = 0; i < K_vector.size(); i++)
  {
    output += String(K_vector[i]) + " "; // Assuming each element has an 'id' field
  }
  output += "d:" + String(dark_offset);
  Serial.println(output); // Print the entire list in one line
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

void BootUp()
{
  /*
      Objective: Handle boot-up procedure. Includes an initial wait, random delay to avoid collision obtained with lux measurement,
                 and broadcast of initial identifier over CAN bus.
  */
  is_waiting = 1;
  delay(25000);
  // is alone?
  unsigned long rand_num = rand() % 100 + 50;
  rand_num = rand_num * 10;
  // Listen for messages
  Serial.println("Ending BootUp");
  Serial.println("rand_num");
  Serial.println(rand_num);
  delay(rand_num); // Listen until end time
                   // if (!rcv_hb_while_wait) {
  snprintf(printbuf, BUFSZ, "LV %d\n", my_id);
  xQueueSend(canMessageQueue, &printbuf, portMAX_DELAY);
  //}

  // Send identifier
  temp_id = rand() % 6000 + 1;
}

void controlLoop(void *pvParameters)
{
  /*
      Objective: Main control loop for reading sensors, computing control values (with consensus),
                 sending/receiving consensus messages, and updating the LED driver.
                 Tracks visibility error, flicker, and energy for performance analysis.
                 Runs indefinitely as a FreeRTOS task.
  */
  while (true)
  {
    unsigned long current_Time = micros();
    float h_time = (current_Time - previous_Time) * 1e-3;
    vTaskDelay(10 / portTICK_PERIOD_MS);
    previous_Time = current_Time;

    if (calib_in_progress == 0 && is_waiting == 2)
    {
      int read_adc;
      float sum_adc = 0;

      // Filter out noise by taking several measurements and computing the average
      for (int i = 0; i < 50; i++)
      {
        read_adc = analogRead(SENSOR_PIN);
        sum_adc += read_adc;
      }
      float avg_adc = sum_adc / 50;

      // Calculate Vldr, Rx, b and luxes
      Vldr = (3.3 / DAC_RANGE) * avg_adc;
      Rx = ((3.3 * 10000) / Vldr) - 10000;
      b = log10(225000) - m;
      lux = pow(10, ((log10(Rx) - b) / m));

      // get external light by getting sensor read and subtracting contributions from own LED
      ext_light = lux - gain * Duty_cycle;
      visibility_error += max(0, reference - lux);

      // specific measurements for obtaining a plot in the report
      if (current_Time >= 24e6 && current_Time <= 28e6)
      {
        visibility_error_aux += max(0, reference - lux);
        N_aux += 1;
      }

      // Calculate b parameter of the feedforward branch in the PI controller, accounting for real-time disturbances
      adjust_PID();

      float jitter = sample_Interval - h_time;

      if (controller == 1)
      {
        float reference_volt = Convert_Volt(reference);
        float u = PID_controller.computeControl(reference_volt, Vldr);

        if (do_consensus)
        {
          if (stage_one)
          {
            if (std::find(u_ready.begin(), u_ready.end(), false) == u_ready.end())
            {
              Serial.printf("AFTER---------------\n");
              std::fill(u_ready.begin(), u_ready.end(), false);
              u_ready[my_id - 1] = true;
              stage_one = true;
              float u____temp = u;
              // Fill node
              node1 = prep_node(node1);

              // Do consensus
              u = consensus_main(node1, u);
              Serial.printf("u____temp: %f u: %f\n", u____temp, u);
            }
            else
            {
              stage_one = false;
              char responseMessage[8] = {0};

              int id_temporary = (int)my_id; // 1 byte: node ID
              // snprintf(responseMessage, sizeof(responseMessage), "%1d %f", id_temporary, u);
              long long tempor = 0;
              tempor = (int)(id_temporary * pow(10, 10));
              Serial.println(tempor);
              tempor = (int)(u * pow(10, 9));

              // memcpy(&id_temporary, &responseMessage[0], sizeof(uint8_t));  // First byte is the node ID
              // responseMessage[1] = ',';
              // memcpy(&u, &responseMessage[2], sizeof(float));  // Next 4 bytes represent the float value
              snprintf(responseMessage, sizeof(responseMessage), "%llu", tempor);
              Serial.printf("responseMessage : %llu, %f\n", tempor, u);
              xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
            }
          }
          else
          {
            if (std::find(u_ready.begin(), u_ready.end(), false) == u_ready.end())
            {
              // Serial.printf("AFTER---------------\n");
              std::fill(u_ready.begin(), u_ready.end(), false);
              u_ready[my_id - 1] = true;
              stage_one = true;
              float u____temp = u;
              // Fill node
              node1 = prep_node(node1);

              // Do consensus
              u = consensus_main(node1, u);
              // Serial.printf("u____temp: %f u: %f\n", u____temp, u);
            }
          }
        }

        // Update LED
        // Serial.printf("u: %f lux: %f\n", u, lux);
        LED_Driver(u, 1);
        Duty_cycle = u;
      }
      else if (controller == 0)
      {
        LED_Driver(0, 1);
      }

      float a = micros();

      if ((Duty_cycle - Duty_cycle1) * (Duty_cycle1 - Duty_cycle2) < 0 && N > 2)
      {
        flicker_error += abs(Duty_cycle - Duty_cycle1) + abs(Duty_cycle1 - Duty_cycle2);
      }

      aux_E += Duty_cycle1 * h_time;
      Duty_cycle2 = Duty_cycle1;
      Duty_cycle1 = Duty_cycle;

      if (print_y == 1)
      {
        Serial.printf("s y %d %f %f\n", LED_id, lux, a / 1e6);
      }
      if (print_u == 1)
      {
        Serial.printf("s u %d %f %f\n", LED_id, Duty_cycle, a / 1e6);
      }
      if (print_y_hub == 1)
      {
        char Message[8] = {0};
        snprintf(Message, sizeof(Message), "LU%f", lux);
        xQueueSend(canMessageQueue, Message, portMAX_DELAY);
      }
      if (print_y_hub == 1)
      {
        char Message[8] = {0};
        snprintf(Message, sizeof(Message), "DC%f", Duty_cycle);
        xQueueSend(canMessageQueue, Message, portMAX_DELAY);
      }

      int aux_dc = Duty_cycle * 10000.0;
      int aux_lux = lux * 100.0;

      float aux_time = current_Time / 1e6;

      buffer1.push_back(aux_dc);
      buffer2.push_back(aux_lux);
      N++;
    }
  }
}

////////////////////////Functions used in previous submission -> Related to the luminaire-level operation

void adjust_PID()
{
  /*
      Objective: control the sistem.
  */
  float reference_volt;
  reference_volt = Convert_Volt(reference);
  H = reference_volt / reference;
  b_pid = -(ext_light / (reference * H * gain * PID_controller.getK())) + (1 / (H * gain * PID_controller.getK())) + 1;
  float b_sat = max(0, min(b_pid, 10000));
  PID_controller.setB(b_sat);
}

void LED_Driver(float value, int flag)
{
  /*
      Args:
        value: the input used to drive the LED.
        flag:  specifies the type of input (0 = lux, 1 = duty cycle [0–1], 2 = percentage [0–100])

      Objective: Drive the LED with the appropriate PWM value depending on the format of `value`.
  */
  if (flag == 0)
  { // direct value [0,DAC_RANGE];
    if (0 <= value <= DAC_RANGE)
    {
      analogWrite(LED_PIN, value);
    }
  }
  if (flag == 1)
  { // duty_cycle [0,1];
    if (0 <= value <= 1)
    {
      analogWrite(LED_PIN, value * DAC_RANGE);
    }
  }
  if (flag == 2)
  { // percentage of LED intensity
    if (0 <= value <= 100)
    {
      analogWrite(LED_PIN, (value / 100) * DAC_RANGE);
    }
  }
}

double readLux()
{
  /*
      Returns: Estimated light intensity in lux

      Objective: Read the analog sensor multiple times to filter noise, read the lux.
  */
  double read_adc = 0;
  double sum_adc = 0;
  for (int i = 0; i < 50; i++)
  {
    read_adc = analogRead(SENSOR_PIN);
    sum_adc += read_adc;
  }
  double avg_adc = sum_adc / 50;
  Vldr = (3.3 / DAC_RANGE) * avg_adc;

  Rx = ((3.3 * 10000) / Vldr) - (10000);
  b = log10(100000) - m;
  lux = pow(10, ((log10(Rx) - b) / m));
  return lux;
}

float Convert_Volt(float lux)
{
  /*
      Args:
        lux: lux

      Returns: voltage (V)

      Objective: Convert a lux value into a voltage.
  */
  float R_lux = pow(10, m * log10(lux) + b);
  float V_lux = (3.3 * 10000) / (R_lux + 10000);
  return V_lux;
}

/////////////////////////////////////////////////////////////////// Consensus functions

// Consensus iterate function in C++
void consensus_iterate(Node &node)
{
  /*
      Args:
        node: Reference to the current node's data.

      Objective: Solve a constrained consensus optimization problem using multiple candidate
                 strategies (unconstrained, boundary-constrained, and linear approximations).
                 Best feasible solution based on minimum cost.
  */
  // Initialization
  std::vector<float> u_best(node.lambda.size(), -1.0); // Start with an invalid solution
  float cost_best = 1000000;                           // Large number for comparison

  std::vector<float> z(node.lambda.size(), 0.0f);
  for (size_t i = 0; i < node.lambda.size(); ++i)
  {
    z[i] = rho * node.u_av - node.lambda[i] - node.c;
  }

  // Unconstrained minimum
  std::vector<float> u_u(node.lambda.size(), 0.0);
  if (check_feasibility(node, u_u))
  {
    float cost_unconstrained = evaluate_cost(node, u_u);
    if (cost_unconstrained < cost_best)
    {
      u_best = u_u;
      cost_best = cost_unconstrained;
      return; //%REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL %NO NEED TO COMPUTE THE OTHER
    }
  }

  // Minimum constrained to linear boundary
  std::vector<float> u_bl(node.lambda.size(), 0.0);

  for (size_t i = 0; i < node.lambda.size(); ++i)
  {
    u_bl[i] = (1.0 / rho) * z[i] - node.k_row[i] / node.n * (node.d - node.L + (1.0 / rho) * z[i] * node.k_row[i]);
  }
  if (check_feasibility(node, u_bl))
  {
    float cost_boundary_linear = evaluate_cost(node, u_bl);
    if (cost_boundary_linear < cost_best)
    {
      u_best = u_bl;
      cost_best = cost_boundary_linear;
    }
  }

  // Minimum constrained to 0 boundary
  std::vector<float> u_b0(node.lambda.size(), 0.0);
  for (size_t i = 0; i < node.lambda.size(); ++i)
  {
    u_b0[i] = (1 / rho) * z[i];
  }
  u_b0[my_id] = 0.0f;
  if (check_feasibility(node, u_b0))
  {
    float cost_boundary_0 = evaluate_cost(node, u_b0);
    if (cost_boundary_0 < cost_best)
    {
      u_best = u_b0;
      cost_best = cost_boundary_0;
    }
  }

  // Minimum constrained to 30 boundary
  std::vector<float> u_b1(node.lambda.size(), 0.0);
  for (size_t i = 0; i < node.lambda.size(); ++i)
  {
    u_b1[i] = (1 / rho) * z[i];
  }
  u_b1[my_id - 1] = 20.0f;
  if (check_feasibility(node, u_b1))
  {
    double cost_boundary_30 = evaluate_cost(node, u_b1);
    if (cost_boundary_30 < cost_best)
    {
      u_best = u_b1;
      cost_best = cost_boundary_30;
    }
  }

  // Minimum constrained to linear and 0 boundary
  std::vector<float> u_l0(node.lambda.size(), 0.0);
  for (size_t i = 0; i < node.lambda.size(); ++i)
  {
    u_l0[i] = (1 / rho) * z[i];
  }
  u_l0[my_id - 1] = 0.0f; // Zero boundary at the index
  // Calculate the required modifications
  float m = 0.0f; // Placeholder for m, replace as needed
  for (size_t i = 0; i < node.lambda.size(); ++i)
  {
    u_l0[i] = (1.0 / rho) * z[i] - (1.0 / node.m) * node.k_row[i] * (node.d - node.L) + (1 / rho / node.m) * node.k_row[i] * (node.k_row[my_id] * z[my_id] - z[i] * node.k_row[i]);
  }
  if (check_feasibility(node, u_l0))
  {
    double cost_linear_0 = evaluate_cost(node, u_l0);
    if (cost_linear_0 < cost_best)
    {
      u_best = u_l0;
      cost_best = cost_linear_0;
    }
  }

  // Minimum constrained to linear and 30 boundary
  std::vector<float> u_l1(node.lambda.size(), 0.0);
  for (size_t i = 0; i < node.lambda.size(); ++i)
  {
    u_l1[i] = (1 / rho) * z[i];
  }
  u_l1[my_id] = 20.0f;
  for (size_t i = 0; i < node.lambda.size(); ++i)
  {
    u_l1[i] = (1.0 / rho) * z[i] - (1.0 / node.m) * node.k_row[i] * (node.d - node.L) + 100 * node.k_row[my_id] + (1 / rho / node.m) * node.k_row[i] * (node.k_row[my_id] * z[my_id] - z[i] * node.k_row[i]);
  }
  if (check_feasibility(node, u_l1))
  {
    double cost_linear_30 = evaluate_cost(node, u_l1);
    if (cost_linear_30 < cost_best)
    {
      u_best = u_l1;
      cost_best = cost_linear_30;
    }
  }

  // Set the best solution and cost
  node.u_vector = u_best;
  node.cost = cost_best;
}

// Consensus main loop
float consensus_main(Node &node, float u)
{
  /*
      Args:
        node: Contains node data.
        u:    Current control value used as a fallback.

      Returns:
        The updated average consensus value.

      Objective:
        Perform one iteration of the distributed consensus algorithm.
        Based on matlab provided in class.
  */

  consensus_iterate(node);

  // check feasibility

  if (!check_feasibility(node, node.u_vector))
  {
    Serial.println("ERROR CONSENSUS");
    return u;
  }

  // Compute the average solution u_av across all nodes
  float temp;
  for (size_t i = 0; i < node.u_vector.size(); ++i)
  {
    temp += node.u_vector[i];
  }

  temp /= node.u_vector.size();

  // Lagrangian updates
  for (size_t i = 0; i < node.u_vector.size(); ++i)
  {
    node.lambda[i] += rho * (node.u_vector[i] - node.u_av);
  }
  return temp;
}

// Cost calculation function (Augmented Lagrangian cost)
float evaluate_cost(Node &node, vector<float> u_vec)
{
  /*
      Args:
        node:   Node
        u_vec:  Candidate control vector to evaluate

      Returns:
        The augmented Lagrangian cost of the candidate control vector

      Objective:
        Calculate a cost function including:
        - Linear term
        - Dual term
        - Quadratic penalty
  */
  float cost = 0.0;

  // First term: c^T u (dot product of c and u)
  for (size_t i = 0; i < u_vec.size(); ++i)
  {
    cost += node.c * u_vec[i]; // Using c_vector as the cost coefficients
  }

  // Second term: lambda^T (u - u_av) (dot product of lambda and (u - u_av))
  for (size_t i = 0; i < u_vec.size(); ++i)
  {
    cost += node.lambda[i] * (u_vec[i] - node.u_av);
  }

  // Third term: (rho / 2) * || u - u_av ||^2 (norm squared of (u - u_av))
  double norm_squared = 0.0;
  for (size_t i = 0; i < u_vec.size(); ++i)
  {
    norm_squared += (u_vec[i] - node.u_av) * (u_vec[i] - node.u_av);
  }
  cost += (rho / 2.0) * norm_squared;

  return cost;
}

// Feasibility check (based on your constraints)
bool check_feasibility(Node &node, vector<float> u_vec)
{
  /*
      Args:
        node:   node containing the data.
        u_vec:  Control vector to validate

      Returns:
        True if the control vector satisfies constraints, False otherwise

      Objective:
        Ensure the solution respects bounds:
        - Local control bounds for this node [0, 100]
        - System-wide linear constraint (e.g. total light ≥ reference - dark_offset)
  */
  float tol = 0.001; // Tolerance for rounding errors

  // Check if the value of u at the node index is outside the bounds [0, 100]
  // SET UP BOUNDS -----------------------------------------------------------------------------------------------------------------------??????????????????
  if (u_vec[my_id - 1] < 0 - tol || u_vec[my_id - 1] > 100 + tol)
  {
    return false;
  }

  // Check if the constraint involving the inner product of u and node.k is violated
  float constraint_value = 0;
  for (size_t i = 0; i < u_vec.size(); ++i)
  {
    constraint_value += u_vec[i] * node.k_row[i];
  }

  if (constraint_value < reference - dark_offset - tol)
  {
    return false;
  }

  return true;
}

// For Set up
void setup()
{
  /*
      Objective:
        Configure hardware interfaces, seed random number generator, initialize task queues,
        and prepare consensus environment (CAN-Bus communication and unique node ID discovery)
  */
  Serial.begin(115200);
  analogReadResolution(12);
  analogWriteFreq(60000);
  analogWriteRange(DAC_RANGE);
  can0.reset();
  can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  can0.setNormalMode();
  pico_get_unique_board_id(&pico_flash_id);
  node_address = pico_flash_id.id[8];
  resourceMutex = xSemaphoreCreateMutex();
  resourceMutex2 = xSemaphoreCreateMutex();
  canMessageQueue = xQueueCreate(1000, sizeof(char) * 8);
  receivedMessageQueue = xQueueCreate(1000, sizeof(struct can_frame));

  // Seed the random number generator with time
  delay(3000);
  int seed_rand = (int)(readLux() * 100000);
  srand(seed_rand);

  vTaskCoreAffinitySet(handle1, (1 << 0));
  vTaskCoreAffinitySet(handle2, (1 << 0));
  vTaskCoreAffinitySet(handle3, (1 << 0));
  vTaskCoreAffinitySet(handle4, (1 << 0));
  vTaskCoreAffinitySet(handle5, (1 << 1));

  xTaskCreate(canReceiveTask, "CAN Receive Task", 1000, NULL, 1, &handle1);
  xTaskCreate(processReceivedMessagesTask, "Process Received Messages Task", 1000, NULL, 1, &handle2);
  xTaskCreate(canSendTask, "CAN Send Task", 1000, NULL, 1, &handle3);
  xTaskCreate(heart_beat, "Heart Beat Task", 1000, NULL, 2, &handle4);
  xTaskCreate(controlLoop, "Control Loop Task", 2000, NULL, 1, &handle5);

  vTaskDelay(pdMS_TO_TICKS(1000));
  // vTaskStartScheduler();

  canMsgTx.can_id = node_address;
  canMsgTx.can_dlc = 8;

  // Send BootUp
  Serial.println("Booting Up\n");

  BootUp();

  if (rcv_hb_while_wait)
  {
    // send requestMessage;
    Serial.println("Requesting ID from other nodes...\n");
    char requestMessage[8] = {0};
    snprintf(requestMessage, sizeof(requestMessage), "R %hu", temp_id);
    xQueueSend(canMessageQueue, &requestMessage, portMAX_DELAY);
    t_last_heartbeat.resize(3);
    prev_run_time = micros();
  }
  else
  {
    my_id = 1;
    luminaire_ids.push_back(my_id);
    Serial.println("I am node 1\n");
    t_last_heartbeat.resize(1);
    prev_run_time = micros();
  }
  is_waiting = 2;
  // vTaskDelay(1000 / portTICK_PERIOD_MS);

  u_ready.resize(luminaire_ids.size());
  std::fill(u_ready.begin(), u_ready.end(), false);
  u_ready[my_id - 1] = true;
}

void print_rcv_cmd(int value1, float value2, int mode)
{
  /*Objective: Print the information requested from other nodes via a Serial Command.*/
  char A_message[10];
  if (mode == 0)
  {
    if (awaiting_R)
    {
      awaiting_R = false;
      if (strncmp("go", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "o %d %d", tgt_node, value1);
        Serial.println(A_message);
      }
      if (strncmp("ga", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "a %d %d", tgt_node, value1);
        Serial.println(A_message);
      }
      if (strncmp("gf", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "f %d %d", tgt_node, value1);
        Serial.println(A_message);
      }
      if (strncmp("gF", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "F %d %d", tgt_node, value1);
        Serial.println(A_message);
      }
      if (strncmp("gc", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "c %d %d", tgt_node, value1);
        Serial.println(A_message);
      }
    }
  }
  else if (mode == 1)
  {
    if (awaiting_R)
    {
      awaiting_R = false;
      if (strncmp("gr", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "r %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gu", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "u %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gg", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "g %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gd", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "d %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gp", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "p %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gt", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "t %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gE", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "E %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gV", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "V %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gQ", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "Q %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gy", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "y %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
      if (strncmp("gv", asked_cmd, 2) == 0)
      {
        snprintf(A_message, sizeof(A_message), "v %d %f", tgt_node, value2);
        Serial.println(A_message);
      }
    }
  }
  strcpy(asked_cmd, "");
  tgt_node = 0;
}

///////////////////////////////////////////////////////////INTERFACE

void print_real_time(float value, int mode)
{
  /*
      Objective: Print the real-time value of the system.
      Args:
        value: The value to be printed.
        mode:  The mode of operation (0 for lux, 1 for duty cycle).
  */
  float a = micros();
  if (mode == 0)
  {
    Serial.printf("s y %d %f %f\n", req_print_id, value, a / 1e6);
  }
  else if (mode == 1)
  {
    Serial.printf("s u %d %f %f\n", req_print_id, value, a / 1e6);
  }
}

void hub(String command)
{
  /*
      Objective: Handle commands received from the serial interface.
      Args:
        command: The command string received from the serial interface.
  */
  if (command.length() > 0)
  {
    char firstchar = command.charAt(0);
    char command_aux[50];
    command.toCharArray(command_aux, 50);
    char auxletter;
    char command2;
    char command3;
    int desk_id;
    float value;
    int temp_val;
    char aux;
    char responseMessage[8] = {0};

    // Special command calibration
    if (command.startsWith("calib"))
    {
      calib_req();
      return;
    }

    switch (firstchar)
    {
    case 'g': // get
      sscanf(command_aux, " %c %c %d ", &auxletter, &command2, &desk_id);
      switch (command2)
      {
      case 'u': // dutycycle
        if (desk_id == my_id)
        {
          Serial.printf("u %d %f\n", desk_id, Duty_cycle);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gu\n");

          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gu");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'g': // gain
        if (desk_id == my_id)
        {
          Serial.printf("g %d %f\n", desk_id, gain);
          print_calib(); // ADDED THIS ONLY FOR HIMSELF
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gg\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gg");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'r': // reference value in luxes
        if (desk_id == my_id)
        {
          Serial.printf("r %d %f\n", desk_id, reference);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gr\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gr");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'y': // luxes read
        if (desk_id == my_id)
        {
          Serial.printf("y %d %f\n", desk_id, lux);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gy\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gy");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'v': // corresponding voltage to the luxes read
        if (desk_id == my_id)
        {
          Serial.printf("v %d %f\n", desk_id, Vldr);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gv\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gv");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'o': // occupancy state (is desk occupied?)
        if (desk_id == my_id)
        {
          Serial.printf("o %d %d\n", desk_id, occupancy);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "go\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "go");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'a': // antiwindup state of PI controller
        if (desk_id == my_id)
        {
          int antiWindup = PID_controller.getAntiWindup();
          Serial.printf("a %d %d\n", desk_id, antiWindup);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "ga\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "ga");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'f': // feedback state of PI controller
        if (desk_id == my_id)
        {
          int feedback = PID_controller.getFeedback();
          Serial.printf("f %d %d\n", desk_id, feedback);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gf\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gf");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'd': // get external light AKA disturbances
        if (desk_id == my_id)
        {
          Serial.printf("d %d %f\n", desk_id, ext_light);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gd\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gd");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'p': // get instataneous power consumed by the LED / desk
        if (desk_id == my_id)
        {
          Serial.printf("p %d %f\n", desk_id, Pmax * Duty_cycle);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gp\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gp");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break; // get current time
      case 't':
        if (desk_id == my_id)
        {
          Serial.printf("t %d %f\n", desk_id, float(micros()) / 1e6);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gt\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gt");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;

      case 'b': // get last minute buffer values
        sscanf(command_aux, " %c %c %c %d ", &auxletter, &command2, &command3, &desk_id);
        if (desk_id == my_id)
        {
          Serial.printf("b %c %d ", command3, desk_id);
          if (command3 == 'u')
          {
            for (int i = buffer1.getTail(); i != buffer1.getHead(); i = (i + 1) % buffer1.getCapacity())
            {
              float aux_val = buffer1.getValue(i) / 1e4;
              Serial.printf("%.4f ,", aux_val);
            }
          }
          else if (command3 == 'y')
          {
            for (int i = buffer2.getTail(); i != buffer2.getHead(); i = (i + 1) % buffer2.getCapacity())
            {
              float aux_val = buffer2.getValue(i) / 1e2;
              Serial.printf("%.2f ,", aux_val);
            }
          }
          Serial.println();
        }
        else
        {
          Serial.printf("err\n");
        }
        break;
      case 'E': // average energy comsuption at desk
        if (desk_id == my_id)
        {
          Serial.printf("E %d %f\n", desk_id, Pmax * (aux_E / N));
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gE\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gE");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'V': // average visibility error
        if (desk_id == my_id)
        {
          Serial.printf("V %d %f\n", desk_id, visibility_error / N);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gV\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gV");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'Q': // average visibility error in a specified interval of time
        if (desk_id == my_id)
        {
          Serial.printf("Q %d %f\n", desk_id, visibility_error_aux / N_aux);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gQ\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gQ");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'F': // average flicker error
        if (desk_id == my_id)
        {
          Serial.printf("F %d %f\n", desk_id, flicker_error / N);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gF\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gF");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      case 'c': // get state of PI controller
        if (desk_id == my_id)
        {
          Serial.printf("c %d %d\n", desk_id, controller);
        }
        else
        {
          awaiting_R = true;
          tgt_node = desk_id;
          strcpy(asked_cmd, "gc\n");
          snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
          snprintf(responseMessage, sizeof(responseMessage), "gc");
          xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        }
        break;
      default:
        Serial.printf("Error - not found\n");
        break;
      }
      break;
    case 'u': // set dutycycle value
      sscanf(command_aux, " %c %d %f", &auxletter, &desk_id, &value);
      if (desk_id == my_id)
      {
        Duty_cycle = value;
        Serial.printf("ack\n");
      }
      else
      {
        snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        snprintf(responseMessage, sizeof(responseMessage), "u%f", value);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
      }
      break;

    case 'a': // set antiwindup state
      sscanf(command_aux, " %c %d %d", &auxletter, &desk_id, &temp_val);
      if (desk_id == my_id)
      {
        PID_controller.setAntiWindup(temp_val);
        Serial.printf("ack\n");
      }
      else
      {
        snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        snprintf(responseMessage, sizeof(responseMessage), "a%d", temp_val);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
      }
      break;
    case 'o': // set occupancy state and change reference accordingly
      sscanf(command_aux, " %c %d %d", &auxletter, &desk_id, &temp_val);
      if (desk_id == my_id)
      {
        if (value == 0)
        {
          occupancy = temp_val;
          reference = 2;
        }
        else if (value == 1)
        {
          occupancy = temp_val;
          reference = 8;
        }
        Serial.printf("ack\n");
      }
      else
      {
        awaiting_R = true;
        tgt_node = desk_id;
        snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        snprintf(responseMessage, sizeof(responseMessage), "o%d", temp_val);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
      }
      break;
    case 's': // print required values
      sscanf(command_aux, " %c %c %d", &auxletter, &aux, &desk_id);
      if (desk_id == my_id)
      {
        if (aux == 'y')
        {
          print_y = 1;
        }
        else if (aux == 'u')
        {
          print_u = 1;
        }
      }
      else
      {
        awaiting_R = true;
        tgt_node = desk_id;
        snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        snprintf(responseMessage, sizeof(responseMessage), "s%c", aux);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        print_RT = 1;           // Print RT
        req_print_id = desk_id; // ID of the node that requested the print
      }
      break;
    case 'S': // stop printing to Serial
      sscanf(command_aux, " %c %c %d", &auxletter, &aux, &desk_id);
      if (desk_id == my_id)
      {
        if (aux == 'y')
        {
          print_y = 0;
        }
        else if (aux == 'u')
        {
          print_u = 0;
        }
        Serial.printf("ack\n");
      }
      else
      {
        awaiting_R = true;
        tgt_node = desk_id;
        snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        snprintf(responseMessage, sizeof(responseMessage), "S%c", aux);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        req_print_id = 0; // ID of the node that requested the print
        print_RT = 0;     // Print RT
      }
      break;
    case 'f': // set feedback state of PI controller
      sscanf(command_aux, " %c %d %d", &auxletter, &desk_id, &temp_val);
      if (desk_id == my_id)
      {
        PID_controller.setFeedback(temp_val);
        Serial.printf("ack\n");
      }
      else
      {
        snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        snprintf(responseMessage, sizeof(responseMessage), "f%d", temp_val);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
      }
      break;
    case 'r': // set reference value in Luxes
      sscanf(command_aux, " %c %d %f", &auxletter, &desk_id, &temp_val);
      if (desk_id == my_id)
      {
        reference = temp_val;
        Serial.printf("ack\n");
      }
      else
      {
        snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        snprintf(responseMessage, sizeof(responseMessage), "r%f", temp_val);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
      }
      break;
    case 'c': // set controller state
      sscanf(command_aux, " %c %d %d", &auxletter, &desk_id, &temp_val);

      if (desk_id == my_id)
      {
        if (temp_val == 0)
        {
          controller = 0;
        }
        else if (temp_val == 1)
        {
          controller = 1;
        }
        Serial.printf("ack\n");
      }
      else
      {
        snprintf(responseMessage, sizeof(responseMessage), "C%d", desk_id);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
        snprintf(responseMessage, sizeof(responseMessage), "c%d", temp_val);
        xQueueSend(canMessageQueue, &responseMessage, portMAX_DELAY);
      }
      break;
    default:
      Serial.println("Error - not found\n");
      break;
    }
  }
  input_Serial = "";
}

////////////////////////////////////////////////////////////
// Main Loop
void loop()
{
  if (Serial.available() > 0)
  {
    input_Serial = Serial.readStringUntil('\n');
    hub(input_Serial);
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
}