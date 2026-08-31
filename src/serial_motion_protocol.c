/**
  * @file  serial_motion_protocol.c
  * @brief 维护串口只解析STOP和ESTOP，不授予任何运动或复位权限
  */

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "serial_motion_protocol.h"

#include <stdio.h>
#include <string.h>

#include "debug_uart_log.h"
#include "debug_uart_transport.h"

#define SERIAL_LINE_SIZE 48U

typedef struct
{
  const char *name;
  motion_command_type_t type;
} serial_command_entry_t;

static const serial_command_entry_t serial_commands[] =
{
  {"STOP", MOTION_CMD_STOP},
  {"ESTOP", MOTION_CMD_ESTOP}
};

static char serial_line[SERIAL_LINE_SIZE];
static uint16_t serial_line_length;
static uint32_t serial_last_sequence;
static uint8_t serial_discard_line;
static motion_command_handler_t serial_handler;
static void *serial_handler_context;

static void serial_ack(uint32_t sequence, const char *name,
                       const char *status, motion_command_error_t error)
{
  debug_uart_log_printf("ACK,%u,%s,%s,%u\r\n", sequence, name, status,
                        (uint32_t)error);
}

static const serial_command_entry_t *serial_find_command(const char *name)
{
  uint8_t index;
  for(index = 0U; index < (uint8_t)(sizeof(serial_commands) /
                                   sizeof(serial_commands[0])); index++)
  {
    if(strcmp(name, serial_commands[index].name) == 0)
    {
      return &serial_commands[index];
    }
  }
  return NULL;
}

void serial_motion_protocol_send_result(const motion_command_t *command,
                                        motion_command_handle_result_t result)
{
  motion_command_error_t error = MOTION_COMMAND_ERROR_NONE;
  const char *status = "REJECTED";
  if(command == NULL)
  {
    return;
  }
  if(result == MOTION_HANDLE_RECEIVED)
  {
    status = "RECEIVED";
  }
  else if(result == MOTION_HANDLE_ACCEPTED)
  {
    status = "ACCEPTED";
  }
  else
  {
    error = (result == MOTION_HANDLE_BUSY) ? MOTION_COMMAND_ERROR_BUSY :
                                             MOTION_COMMAND_ERROR_UNSUPPORTED;
  }
  serial_ack(command->sequence, motion_command_get_name(command->type),
             status, error);
}

static void serial_parse_line(char *line)
{
  char name[8];
  char extra;
  unsigned long version;
  unsigned long sequence;
  const serial_command_entry_t *entry;
  motion_command_t command;
  motion_command_error_t error;

  if(sscanf(line, "CMD,%7[^,],%lu,%lu%c", name, &version, &sequence,
            &extra) != 3)
  {
    serial_ack(0U, "UNKNOWN", "REJECTED", MOTION_COMMAND_ERROR_FORMAT);
    return;
  }
  entry = serial_find_command(name);
  if(entry == NULL)
  {
    serial_ack((uint32_t)sequence, name, "REJECTED",
               MOTION_COMMAND_ERROR_UNSUPPORTED);
    return;
  }
  memset(&command, 0, sizeof(command));
  command.version = (uint8_t)version;
  command.sequence = (uint32_t)sequence;
  command.source = MOTION_SOURCE_SERIAL;
  command.type = entry->type;
  if(version > 255UL)
  {
    error = MOTION_COMMAND_ERROR_FORMAT;
  }
  else if((sequence == 0UL) || (sequence > 65535UL))
  {
    error = MOTION_COMMAND_ERROR_SEQUENCE;
  }
  else
  {
    error = motion_command_validate(&command);
  }
  if(error != MOTION_COMMAND_ERROR_NONE)
  {
    serial_ack(command.sequence, entry->name, "REJECTED", error);
    return;
  }
  if(command.sequence == serial_last_sequence)
  {
    serial_ack(command.sequence, entry->name, "DUPLICATE",
               MOTION_COMMAND_ERROR_NONE);
    return;
  }
  if(serial_handler == NULL)
  {
    serial_ack(command.sequence, entry->name, "REJECTED",
               MOTION_COMMAND_ERROR_UNSUPPORTED);
    return;
  }
  serial_last_sequence = command.sequence;
  serial_motion_protocol_send_result(
      &command, serial_handler(&command, serial_handler_context));
}

void serial_motion_protocol_init(motion_command_handler_t handler,
                                 void *context)
{
  serial_line_length = 0U;
  serial_discard_line = 0U;
  serial_last_sequence = 0U;
  serial_handler = handler;
  serial_handler_context = context;
}

void serial_motion_protocol_poll(void)
{
  uint8_t data;
  if(debug_uart_transport_take_overflow() != 0U)
  {
    serial_line_length = 0U;
    serial_discard_line = 1U;
    debug_uart_log_printf("RXERR,RING_OVERFLOW\r\n");
  }
  while(debug_uart_transport_read_byte(&data) != 0U)
  {
    if(data == '\r')
    {
      continue;
    }
    if(data == '\n')
    {
      if(serial_discard_line != 0U)
      {
        serial_discard_line = 0U;
        debug_uart_log_printf("RXERR,LINE_TOO_LONG\r\n");
      }
      else if(serial_line_length != 0U)
      {
        serial_line[serial_line_length] = '\0';
        serial_parse_line(serial_line);
      }
      serial_line_length = 0U;
      continue;
    }
    if(serial_discard_line != 0U)
    {
      continue;
    }
    if(serial_line_length >= (SERIAL_LINE_SIZE - 1U))
    {
      serial_line_length = 0U;
      serial_discard_line = 1U;
      continue;
    }
    serial_line[serial_line_length++] = (char)data;
  }
}
