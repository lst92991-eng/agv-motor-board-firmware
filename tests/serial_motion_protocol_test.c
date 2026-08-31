#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "serial_motion_protocol.h"

static const char *input_text;
static uint16_t input_index;
static char last_log[256];
static motion_command_t captured_command;
static uint16_t handled_count;
static int failures;

uint8_t debug_uart_transport_read_byte(uint8_t *data)
{
  if((data == NULL) || (input_text == NULL) ||
     (input_text[input_index] == '\0'))
  {
    return 0U;
  }
  *data = (uint8_t)input_text[input_index++];
  return 1U;
}

uint8_t debug_uart_transport_take_overflow(void)
{
  return 0U;
}

int debug_uart_log_printf(const char *format, ...)
{
  va_list arguments;
  int result;

  va_start(arguments, format);
  result = vsnprintf(last_log, sizeof(last_log), format, arguments);
  va_end(arguments);
  return result;
}

static motion_command_handle_result_t handle_command(
    const motion_command_t *command,
    void *context)
{
  (void)context;
  captured_command = *command;
  handled_count++;
  return MOTION_HANDLE_ACCEPTED;
}

static void expect_true(int condition, const char *message)
{
  if(!condition)
  {
    printf("FAIL,%s\n", message);
    failures++;
  }
}

static void run_line(const char *line)
{
  input_text = line;
  input_index = 0U;
  last_log[0] = '\0';
  serial_motion_protocol_poll();
}

static void reset_protocol(void)
{
  memset(&captured_command, 0, sizeof(captured_command));
  handled_count = 0U;
  serial_motion_protocol_init(handle_command, NULL);
}

static void test_emergency_commands_are_allowed(void)
{
  reset_protocol();
  run_line("CMD,STOP,1,1\n");
  expect_true(handled_count == 1U &&
                  captured_command.type == MOTION_CMD_STOP &&
                  captured_command.source == MOTION_SOURCE_SERIAL,
              "maintenance UART accepts STOP");

  reset_protocol();
  run_line("CMD,ESTOP,1,2\n");
  expect_true(handled_count == 1U &&
                  captured_command.type == MOTION_CMD_ESTOP,
              "maintenance UART accepts ESTOP");
}

static void test_active_and_reset_commands_are_rejected(void)
{
  static const char *commands[] =
  {
    "CMD,GOTO,1,3,300,0,0,50\n",
    "CMD,CLEAR_FAULT,1,4\n",
    "CMD,RESET_ODOM,1,5\n",
    "CMD,FOLLOW_MAG,1,6\n"
  };
  uint16_t index;

  for(index = 0U; index <
      (uint16_t)(sizeof(commands) / sizeof(commands[0])); index++)
  {
    reset_protocol();
    run_line(commands[index]);
    expect_true(handled_count == 0U &&
                    strstr(last_log, "REJECTED") != NULL,
                "maintenance UART rejects active authority");
  }
}

static void test_session_release_requires_control_confirmation(void)
{
  expect_true(motion_command_releases_session(
                  MOTION_CMD_RELEASE, MOTION_HANDLE_RECEIVED) == 0U,
              "queued release does not close session early");
  expect_true(motion_command_releases_session(
                  MOTION_CMD_RELEASE, MOTION_HANDLE_BUSY) == 0U,
              "busy release keeps session protection");
  expect_true(motion_command_releases_session(
                  MOTION_CMD_RELEASE, MOTION_HANDLE_ACCEPTED) != 0U,
              "accepted release closes session");
  expect_true(motion_command_releases_session(
                  MOTION_CMD_DISABLE, MOTION_HANDLE_ACCEPTED) == 0U,
              "disable keeps the owned session");
}

int main(void)
{
  test_emergency_commands_are_allowed();
  test_active_and_reset_commands_are_rejected();
  test_session_release_requires_control_confirmation();
  if(failures != 0)
  {
    printf("SERIAL_MOTION_PROTOCOL_TEST_FAIL,%d\n", failures);
    return 1;
  }
  puts("SERIAL_MOTION_PROTOCOL_TEST_PASS");
  return 0;
}
