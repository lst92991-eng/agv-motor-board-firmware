/**
  *****************************************************************************
  * @file     serial_motion_protocol.c
  * @brief    维护串口紧急停车命令解析与ACK应答
  *****************************************************************************
  */

#include "serial_motion_protocol.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug_uart_log.h"
#include "debug_uart_transport.h"

#define SERIAL_MOTION_LINE_SIZE                 192U
#define SERIAL_MOTION_MAX_FIELDS                  4U

static char serial_motion_line[SERIAL_MOTION_LINE_SIZE];
static uint16_t serial_motion_line_length;
static uint8_t serial_motion_discard_line;
static uint32_t serial_motion_last_sequence;
static motion_command_handler_t serial_motion_handler;
static void *serial_motion_handler_context;

static uint8_t serial_motion_parse_long(const char *text,
                                        long value_min,
                                        long value_max,
                                        long *value)
{
  char *end_ptr;
  long parsed_value;

  if((text == 0) || (*text == '\0') || (value == 0))
  {
    return 0U;
  }

  errno = 0;
  parsed_value = strtol(text, &end_ptr, 10);
  if((errno != 0) || (*end_ptr != '\0') ||
     (parsed_value < value_min) || (parsed_value > value_max))
  {
    return 0U;
  }

  *value = parsed_value;
  return 1U;
}

static uint8_t serial_motion_split_fields(char *line,
                                          char **fields,
                                          uint8_t field_capacity)
{
  uint8_t field_count = 0U;
  char *cursor;

  if((line == 0) || (fields == 0) || (field_capacity == 0U))
  {
    return 0U;
  }

  fields[field_count++] = line;
  cursor = line;
  while(*cursor != '\0')
  {
    if(*cursor == ',')
    {
      *cursor = '\0';
      if(field_count >= field_capacity)
      {
        return 0U;
      }
      fields[field_count++] = cursor + 1;
    }
    cursor++;
  }
  return field_count;
}

static void serial_motion_send_ack(uint32_t sequence,
                                   const char *command_name,
                                   const char *status,
                                   motion_command_error_t error)
{
  debug_uart_log_printf("ACK,%u,%s,%s,%u\r\n",
                        (uint32_t)sequence,
                        command_name,
                        status,
                        (uint32_t)error);
}

void serial_motion_protocol_send_result(
    const motion_command_t *command,
    motion_command_handle_result_t result)
{
  const char *command_name = motion_command_get_name(command->type);

  if(result == MOTION_HANDLE_RECEIVED)
  {
    serial_motion_send_ack(command->sequence, command_name, "RECEIVED",
                           MOTION_COMMAND_ERROR_NONE);
  }
  else if(result == MOTION_HANDLE_ACCEPTED)
  {
    serial_motion_send_ack(command->sequence, command_name, "ACCEPTED",
                           MOTION_COMMAND_ERROR_NONE);
  }
  else if(result == MOTION_HANDLE_BUSY)
  {
    serial_motion_send_ack(command->sequence, command_name, "REJECTED",
                           MOTION_COMMAND_ERROR_BUSY);
  }
  else
  {
    serial_motion_send_ack(command->sequence, command_name, "REJECTED",
                           MOTION_COMMAND_ERROR_UNSUPPORTED);
  }
}

static void serial_motion_parse_line(char *line)
{
  char *fields[SERIAL_MOTION_MAX_FIELDS];
  uint8_t field_count;
  long parsed_value;
  motion_command_t command;
  motion_command_error_t error;
  motion_command_handle_result_t handle_result;

  memset(&command, 0, sizeof(command));
  command.source = MOTION_SOURCE_SERIAL;

  field_count = serial_motion_split_fields(line, fields,
                                           SERIAL_MOTION_MAX_FIELDS);
  if((field_count != 4U) || (strcmp(fields[0], "CMD") != 0))
  {
    serial_motion_send_ack(0U, "UNKNOWN", "REJECTED",
                           MOTION_COMMAND_ERROR_FORMAT);
    return;
  }

  if(serial_motion_parse_long(fields[2], 0L, 255L, &parsed_value) == 0U)
  {
    serial_motion_send_ack(0U, fields[1], "REJECTED",
                           MOTION_COMMAND_ERROR_FORMAT);
    return;
  }
  command.version = (uint8_t)parsed_value;

  if(serial_motion_parse_long(fields[3], 1L, 65535L, &parsed_value) == 0U)
  {
    serial_motion_send_ack(0U, fields[1], "REJECTED",
                           MOTION_COMMAND_ERROR_SEQUENCE);
    return;
  }
  command.sequence = (uint32_t)parsed_value;

  if(strcmp(fields[1], "STOP") == 0)
  {
    command.type = MOTION_CMD_STOP;
  }
  else if(strcmp(fields[1], "ESTOP") == 0)
  {
    command.type = MOTION_CMD_ESTOP;
  }
  else
  {
    /* Production UART never grants motion, reset or fault-clear authority. */
    serial_motion_send_ack(command.sequence, fields[1], "REJECTED",
                           MOTION_COMMAND_ERROR_UNSUPPORTED);
    return;
  }

  error = motion_command_validate(&command);
  if(error != MOTION_COMMAND_ERROR_NONE)
  {
    serial_motion_send_ack(command.sequence,
                           motion_command_get_name(command.type),
                           "REJECTED", error);
    return;
  }

  if(command.sequence == serial_motion_last_sequence)
  {
    serial_motion_send_ack(command.sequence,
                           motion_command_get_name(command.type),
                           "DUPLICATE", MOTION_COMMAND_ERROR_NONE);
    return;
  }

  if(serial_motion_handler == 0)
  {
    serial_motion_send_ack(command.sequence,
                           motion_command_get_name(command.type),
                           "REJECTED", MOTION_COMMAND_ERROR_UNSUPPORTED);
    return;
  }

  serial_motion_last_sequence = command.sequence;
  handle_result = serial_motion_handler(&command,
                                        serial_motion_handler_context);
  serial_motion_protocol_send_result(&command, handle_result);
}

void serial_motion_protocol_init(motion_command_handler_t handler,
                                 void *context)
{
  serial_motion_line_length = 0U;
  serial_motion_discard_line = 0U;
  serial_motion_last_sequence = 0U;
  serial_motion_handler = handler;
  serial_motion_handler_context = context;
}

void serial_motion_protocol_poll(void)
{
  uint8_t data;

  if(debug_uart_transport_take_overflow() != 0U)
  {
    serial_motion_line_length = 0U;
    serial_motion_discard_line = 1U;
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
      if(serial_motion_discard_line != 0U)
      {
        serial_motion_discard_line = 0U;
        serial_motion_line_length = 0U;
        debug_uart_log_printf("RXERR,LINE_TOO_LONG\r\n");
      }
      else if(serial_motion_line_length != 0U)
      {
        serial_motion_line[serial_motion_line_length] = '\0';
        serial_motion_parse_line(serial_motion_line);
        serial_motion_line_length = 0U;
      }
      continue;
    }

    if(serial_motion_discard_line != 0U)
    {
      continue;
    }
    if(serial_motion_line_length >= (SERIAL_MOTION_LINE_SIZE - 1U))
    {
      serial_motion_line_length = 0U;
      serial_motion_discard_line = 1U;
      continue;
    }
    serial_motion_line[serial_motion_line_length++] = (char)data;
  }
}
