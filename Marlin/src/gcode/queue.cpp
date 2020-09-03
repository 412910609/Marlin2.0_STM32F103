/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * queue.cpp - The G-code command queue
 */

#include "queue.h"
GCodeQueue queue;

#include "gcode.h"

#include "../lcd/ultralcd.h"
#include "../sd/cardreader.h"
#include "../module/planner.h"
#include "../module/temperature.h"
#include "../Marlin.h"

#if ENABLED(PRINTER_EVENT_LEDS)
#include "../feature/leds/printer_event_leds.h"
#endif

#if ENABLED(BINARY_FILE_TRANSFER)
#include "../feature/binary_protocol.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
#include "../feature/power_loss_recovery.h"
#endif

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
long gcode_N, GCodeQueue::last_N, GCodeQueue::stopped_N = 0, GCodeQueue::last_P;
uint16_t GCodeQueue::cmd_length = 0;

/**
 * GCode Command Queue
 * A simple ring buffer of BUFSIZE command strings.
 *
 * Commands are copied into this buffer by the command injectors
 * (immediate, serial, sd card) and they are processed sequentially by
 * the main loop. The gcode.process_next_command method parses the next
 * command and hands off execution to individual handler functions.
 */
uint8_t GCodeQueue::length = 0, // Count of commands in the queue
    GCodeQueue::index_r = 0,    // Ring buffer read position
    GCodeQueue::index_w = 0;    // Ring buffer write position

char GCodeQueue::command_buffer[BUFSIZE][MAX_CMD_SIZE];
GCodeQueue::PacketState GCodeQueue::packet_state = GCodeQueue::PacketState::HEAD;

/*
 * The port that the command was received on
 */
#if NUM_SERIAL > 1
int16_t GCodeQueue::port[BUFSIZE];
#endif

/**
 * Serial command injection
 */

// Number of characters read in the current line of serial input
static int serial_count[NUM_SERIAL] = {0};

bool send_ok[BUFSIZE];

/**
 * Next Injected Command pointer. nullptr if no commands are being injected.
 * Used by Marlin internally to ensure that commands initiated from within
 * are enqueued ahead of any pending serial or sd card commands.
 */
static PGM_P injected_commands_P = nullptr;

GCodeQueue::GCodeQueue()
{
  // Send "ok" after commands by default
  for (uint8_t i = 0; i < COUNT(send_ok); i++)
    send_ok[i] = true;
}

/**
 * Check whether there are any commands yet to be executed
 */
bool GCodeQueue::has_commands_queued()
{
  return queue.length || injected_commands_P;
}

/**
 * Clear the Marlin command queue
 */
void GCodeQueue::clear()
{
  index_r = index_w = length = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
void GCodeQueue::_commit_command(bool say_ok
#if NUM_SERIAL > 1
                                 ,
                                 int16_t p /*=-1*/
#endif
)
{
  send_ok[index_w] = say_ok;
#if NUM_SERIAL > 1
  port[index_w] = p;
#endif
#if ENABLED(POWER_LOSS_RECOVERY)
  recovery.commit_sdpos(index_w);
#endif
  if (++index_w >= BUFSIZE)
    index_w = 0;
  length++;
}

/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
bool GCodeQueue::_enqueue(const char *cmd, bool say_ok /*=false*/
#if NUM_SERIAL > 1
                          ,
                          int16_t pn /*=-1*/
#endif
)
{
  if (*cmd == ';' || length >= BUFSIZE)
    return false;
  strcpy(command_buffer[index_w], cmd);
  _commit_command(say_ok
#if NUM_SERIAL > 1
                  ,
                  pn
#endif
  );
  return true;
}

/**
 * Enqueue with Serial Echo
 * Return true if the command was consumed
 */
bool GCodeQueue::enqueue_one(const char *cmd)
{

  //SERIAL_ECHOPGM("enqueue_one(\"");
  //SERIAL_ECHO(cmd);
  //SERIAL_ECHOPGM("\") \n");

  if (*cmd == 0 || *cmd == '\n' || *cmd == '\r')
    return true;

  if (_enqueue(cmd))
  {
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPAIR(MSG_ENQUEUEING, cmd, "\"");
    return true;
  }
  return false;
}

/**
 * Process the next "immediate" command.
 * Return 'true' if any commands were processed,
 * or remain to process.
 */
bool GCodeQueue::process_injected_command()
{
  if (injected_commands_P == nullptr)
    return false;

  char c;
  size_t i = 0;
  while ((c = pgm_read_byte(&injected_commands_P[i])) && c != '\n')
    i++;

  // Extract current command and move pointer to next command
  char cmd[i + 1];
  memcpy_P(cmd, injected_commands_P, i);
  cmd[i] = '\0';
  injected_commands_P = c ? injected_commands_P + i + 1 : nullptr;

  // Execute command if non-blank
  if (i)
  {
    parser.parse(cmd);
    PORT_REDIRECT(SERIAL_PORT);
    gcode.process_parsed_command();
  }
  return true;
}

/**
 * Enqueue one or many commands to run from program memory.
 * Do not inject a comment or use leading spaces!
 * Aborts the current queue, if any.
 * Note: process_injected_command() will be called to drain any commands afterwards
 */
void GCodeQueue::inject_P(PGM_P const pgcode) { injected_commands_P = pgcode; }

/**
 * Enqueue and return only when commands are actually enqueued.
 * Never call this from a G-code handler!
 */
void GCodeQueue::enqueue_one_now(const char *cmd)
{
  while (!enqueue_one(cmd))
    idle();
}

/**
 * Enqueue from program memory and return only when commands are actually enqueued
 * Never call this from a G-code handler!
 */
void GCodeQueue::enqueue_now_P(PGM_P const pgcode)
{
  size_t i = 0;
  PGM_P p = pgcode;
  for (;;)
  {
    char c;
    while ((c = pgm_read_byte(&p[i])) && c != '\n')
      i++;
    char cmd[i + 1];
    memcpy_P(cmd, p, i);
    cmd[i] = '\0';
    enqueue_one_now(cmd);
    if (!c)
      break;
    p += i + 1;
  }
}

/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 *
 * If ADVANCED_OK is enabled also include:
 *   N<int>  Line number of the command, if any
 *   P<int>  Planner space remaining
 *   B<int>  Block queue space remaining
 */
void GCodeQueue::ok_to_send()
{
#if NUM_SERIAL > 1
  const int16_t pn = port[index_r];
  if (pn < 0)
    return;
  PORT_REDIRECT(pn);
#endif
  if (!send_ok[index_r])
    return;
  SERIAL_ECHOPGM(MSG_OK);
#if ENABLED(ADVANCED_OK)
  char *p = command_buffer[index_r];
  if (*p == 'N')
  {
    SERIAL_ECHO(' ');
    SERIAL_ECHO(*p++);
    while (NUMERIC_SIGNED(*p))
      SERIAL_ECHO(*p++);
  }
  SERIAL_ECHOPGM(" P");
  SERIAL_ECHO(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
  SERIAL_ECHOPGM(" B");
  SERIAL_ECHO(BUFSIZE - length);
#endif
  SERIAL_EOL();
}

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void GCodeQueue::flush_and_request_resend()
{
#if NUM_SERIAL > 1
  const int16_t p = port[index_r];
  if (p < 0)
    return;
  PORT_REDIRECT(p);
#endif
  SERIAL_FLUSH();
  SERIAL_ECHOPGM(MSG_RESEND);
  SERIAL_ECHOLN(last_N + 1);
  ok_to_send();
}

inline bool serial_data_available()
{
  return false || MYSERIAL0.available()
#if NUM_SERIAL > 1
         || MYSERIAL1.available()
#endif
      ;
}

inline int read_serial(const uint8_t index)
{
  switch (index)
  {
  case 0:
    return MYSERIAL0.read();
#if NUM_SERIAL > 1
  case 1:
    return MYSERIAL1.read();
#endif
  default:
    return -1;
  }
}

void GCodeQueue::gcode_line_error(PGM_P const err, const int8_t port)
{
  PORT_REDIRECT(port);
  SERIAL_ERROR_START();
  serialprintPGM(err);
  SERIAL_ECHOLN(last_N);
  while (read_serial(port) != -1)
    ; // clear out the RX buffer
  flush_and_request_resend();
  serial_count[port] = 0;
}

void GCodeQueue::gcode_packet_error(PGM_P const err, const int8_t port)
{
  PORT_REDIRECT(port);
  SERIAL_ERROR_START();
  serialprintPGM(err);
  SERIAL_ECHOLN(last_P);
  while (read_serial(port) != -1)
    ; // clear out the RX buffer
  flush_and_request_resend();
  serial_count[port] = 0;
}

FORCE_INLINE bool is_M29(const char *const cmd)
{ // matches "M29" & "M29 ", but not "M290", etc
  const char *const m29 = strstr_P(cmd, PSTR("M29"));
  return m29 && !NUMERIC(m29[3]);
}

/**
 * Get all commands waiting on the serial port and queue them.
 * Exit when the buffer is full or when no more characters are
 * left on the serial port.
 */
void GCodeQueue::get_serial_commands()
{
  static char serial_line_buffer[NUM_SERIAL][MAX_CMD_SIZE];
  static bool serial_comment_mode[NUM_SERIAL] = { false }
#if ENABLED(PAREN_COMMENTS)
  ,
              serial_comment_paren_mode[NUM_SERIAL] = { false }
#endif
  ;
  //串口数据包接收程序
  //方案1：协议为：有效数据(MAX_CMD_SIZE-6) + 偏移地址(4bytes) + 校验(1byte) + 结束符0x83(1byte)

  // if (card.flag.saving)
  // {
  //   while (serial_data_available())
  //   {
  //     for (uint8_t i = 0; i < NUM_SERIAL; ++i)
  //     {
  //       int c;
  //       if ((c = read_serial(i)) < 0)
  //         continue;
  //       char serial_char = c;
  //       SERIAL_CHAR(serial_char); //Test: 字符回显
  //       /**
  //      * If the character ends the data packet
  //      */
  //       if (serial_char == (char)0x83)
  //       {
  //         SERIAL_ECHOLNPGM("Print in 0x83 ");
  //         serial_line_buffer[i][serial_count[i]] = 0; // Terminate string
  //         char *command = serial_line_buffer[i];
  //         if (is_M29(command))
  //         {
  //           //写文件结束，接收到M29命令，关闭文件
  //           card.closefile();
  //           SERIAL_ECHOLNPGM(MSG_FILE_SAVED);
  //           last_P = 0;
  //           serial_count[i] = 0; // Reset buffer
  //           ok_to_send();
  //           return;
  //         }
  //         uint8_t parity = uint8_t(serial_line_buffer[i][serial_count[i] - 1]); //获取校验 1byte
  //         serial_line_buffer[i][serial_count[i] - 1] = 0;                       // Terminate string
  //         // char *command = serial_line_buffer[i];
  //         serialprintPGM(command); //Test: 0x83字符串回显
  //         char *apos = &serial_line_buffer[i][serial_count[i] - 5];
  //         SERIAL_ECHOLNPAIR("RX parity is:", parity);
  //         int32_t p_offset = strtol(apos, nullptr, 10);
  //         SERIAL_ECHOLNPAIR("RX p_offset is:", p_offset);
  //         SERIAL_ECHOLNPAIR("data offset is:", last_P);
  //         if (p_offset != last_P) //校验写地址
  //           return gcode_packet_error("offset error, last offset:", i);
  //         uint8_t checksum = 0, count = strlen(command);
  //         SERIAL_ECHOLNPAIR("cmd length is:", count);
  //         while (count)
  //           checksum ^= command[--count];
  //         SERIAL_ECHOLNPAIR("checksum is:", checksum);
  //         if (parity != checksum) //校验校验位
  //           return gcode_packet_error("checksum error, last offset:", i);
  //         serial_count[i] = 0; // Reset buffer
  //         int16_t wrire_cnt;
  //         wrire_cnt = card.write_stream(command, strlen(command) - 4);
  //         if (wrire_cnt > 0)
  //         {
  //           last_P += wrire_cnt;
  //           SERIAL_ECHOLNPAIR("write done:", last_P);
  //           ok_to_send();
  //         }
  //         else
  //           return gcode_packet_error("Error: write data!, last offset:", i);
  //       }
  //       else if (serial_count[i] >= MAX_CMD_SIZE - 1)
  //         return gcode_packet_error("over size, last offset:", i);
  //       else
  //       { // it's not a newline, carriage return or escape char
  //         serial_line_buffer[i][serial_count[i]++] = serial_char;
  //       }
  //     }
  //   }
  //   return;
  // }

  //串口数据包接收程序
  //方案2：协议为：帧头0x55AA(2bytes) + 数据长度(4bytes) + 有效数据(MAX_CMD_SIZE) + 校验(1byte)
  if (card.flag.saving)
  {
    while (serial_data_available())
    {
      for (uint8_t i = 0; i < NUM_SERIAL; ++i)
      {
        int c;
        if ((c = read_serial(i)) < 0)
          continue;

        char serial_char = c;
        //SERIAL_CHAR(serial_char); //Test: 字符回显
        //SERIAL_ECHOLNPAIR("packet_state is:", uint8_t(packet_state));
        /**
       * If the character ends the data packet
       */
        char *command;
        uint8_t checksum = 0;
        uint16_t count = 0;

        switch (packet_state)
        {
          /**
          * Data packet handling
          */
        case PacketState::HEAD:
          if ((uint8_t(serial_char) == PACKET_HEAD2) && (uint8_t(serial_line_buffer[i][serial_count[i] - 1]) == PACKET_HEAD1))
          {
            serial_count[i] = 0; // Reset buffer
            packet_state = PacketState::LENGTH;
          }
          else if (serial_char == '\n' || serial_char == '\r')
          {
            // Skip empty lines and comments
            if (!serial_count[i])
              continue;

            serial_line_buffer[i][serial_count[i]] = 0; // Terminate string
            serial_count[i] = 0;                        // Reset buffer

            command = serial_line_buffer[i];
            if (is_M29(command))
            {
              // M29 closes the file
              card.closefile();
              SERIAL_ECHOLNPGM(MSG_FILE_SAVED);

              ok_to_send();
              return;
            }
          }
          else
            serial_line_buffer[i][serial_count[i]++] = serial_char;
          break;
        case PacketState::LENGTH:
          if (serial_count[i] >= 4)
          {
            serial_line_buffer[i][serial_count[i]] = 0; // Terminate string
            serial_count[i] = 0;                        // Reset buffer

            cmd_length = strtol(serial_line_buffer[i], nullptr, 10);
            //SERIAL_ECHOLNPAIR("cmd_length is:", cmd_length);
            if (cmd_length > MAX_CMD_SIZE)
            {
              SERIAL_ERROR_MSG("The length out of range! Note: MAX_CMD_SIZE is");
              SERIAL_ECHOLNPAIR(" ", MAX_CMD_SIZE);
              packet_state = PacketState::HEAD;
              return;
            }
            packet_state = PacketState::DATA;
          }
          serial_line_buffer[i][serial_count[i]++] = serial_char; //在这里会接收Data的第一个字符
          break;
        case PacketState::DATA:
          if (--cmd_length)
          {
            serial_line_buffer[i][serial_count[i]++] = serial_char;
            break;
          }
          cmd_length = 0;
          serial_line_buffer[i][serial_count[i]] = 0; // Terminate string
          serial_count[i] = 0;                        // Reset buffer
          packet_state = PacketState::CHECK;
        case PacketState::CHECK:
          command = serial_line_buffer[i];
          //SERIAL_ECHOLNPGM("Received data:");
          //serialprintPGM(command); //Test:接收到的数据回显
          count = strlen(command);
          //SERIAL_ECHOLNPAIR("count is:", count);
          while (count)
            checksum ^= command[--count];
          //SERIAL_ECHOLNPAIR("checksum is:", checksum);
          //SERIAL_ECHOLNPAIR("Rx check is:", uint8_t(serial_char));
          if (checksum != uint8_t(serial_char))
          {
            SERIAL_ECHO_MSG("Data checksum is failed!");
            packet_state = PacketState::HEAD;
            return;
          }
          else
            packet_state = PacketState::WRITE;
        case PacketState::WRITE:
          card.write_packet(serial_line_buffer[i]);
          ok_to_send();
          packet_state = PacketState::HEAD;
          break;
        }
      }
    }
    return;
  }

#if ENABLED(BINARY_FILE_TRANSFER)
  if (card.flag.binary_mode)
  {
    /**
       * For binary stream file transfer, use serial_line_buffer as the working
       * receive buffer (which limits the packet size to MAX_CMD_SIZE).
       * The receive buffer also limits the packet size for reliable transmission.
       */
    binaryStream[card.transfer_port_index].receive(serial_line_buffer[card.transfer_port_index]);
    return;
  }
#endif

// If the command buffer is empty for too long,
// send "wait" to indicate Marlin is still waiting.
#if NO_TIMEOUTS > 0
  static millis_t last_command_time = 0;
  const millis_t ms = millis();
  if (length == 0 && !serial_data_available() && ELAPSED(ms, last_command_time + NO_TIMEOUTS))
  {
    SERIAL_ECHOLNPGM(MSG_WAIT);
    last_command_time = ms;
  }
#endif

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (length < BUFSIZE && serial_data_available())
  {
    for (uint8_t i = 0; i < NUM_SERIAL; ++i)
    {
      int c;
      if ((c = read_serial(i)) < 0)
        continue;

      char serial_char = c;

      /**
       * If the character ends the line
       */
      if (serial_char == '\n' || serial_char == '\r')
      {

        // Start with comment mode off
        serial_comment_mode[i] = false;
#if ENABLED(PAREN_COMMENTS)
        serial_comment_paren_mode[i] = false;
#endif

        // Skip empty lines and comments
        if (!serial_count[i])
        {
          //thermalManager.manage_heater();
          continue;
        }

        serial_line_buffer[i][serial_count[i]] = 0; // Terminate string
        serial_count[i] = 0;                        // Reset buffer

        char *command = serial_line_buffer[i];

        while (*command == ' ')
          command++;                                        // Skip leading spaces
        char *npos = (*command == 'N') ? command : nullptr; // Require the N parameter to start the line

        if (npos)
        {

          bool M110 = strstr_P(command, PSTR("M110")) != nullptr;

          if (M110)
          {
            char *n2pos = strchr(command + 4, 'N');
            if (n2pos)
              npos = n2pos;
          }

          gcode_N = strtol(npos + 1, nullptr, 10);

          if (gcode_N != last_N + 1 && !M110)
            return gcode_line_error(PSTR(MSG_ERR_LINE_NO), i);

          char *apos = strrchr(command, '*');
          if (apos)
          {
            uint8_t checksum = 0, count = uint8_t(apos - command);
            while (count)
              checksum ^= command[--count];
            if (strtol(apos + 1, nullptr, 10) != checksum)
              return gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH), i);
          }
          else
            return gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM), i);

          last_N = gcode_N;
        }
#if ENABLED(SDSUPPORT)
        // Pronterface "M29" and "M29 " has no line number
        else if (card.flag.saving && !is_M29(command))
          return gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM), i);
#endif

        // Movement commands alert when stopped
        if (IsStopped())
        {
          char *gpos = strchr(command, 'G');
          if (gpos)
          {
            switch (strtol(gpos + 1, nullptr, 10))
            {
            case 0:
            case 1:
#if ENABLED(ARC_SUPPORT)
            case 2:
            case 3:
#endif
#if ENABLED(BEZIER_CURVE_SUPPORT)
            case 5:
#endif
              SERIAL_ECHOLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
            }
          }
        }

#if DISABLED(EMERGENCY_PARSER)
        // Process critical commands early
        if (strcmp(command, "M108") == 0)
        {
          wait_for_heatup = false;
#if HAS_LCD_MENU
          wait_for_user = false;
#endif
        }
        if (strcmp(command, "M112") == 0)
          kill();
        if (strcmp(command, "M410") == 0)
          quickstop_stepper();
#endif

#if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
#endif

        // Add the command to the queue
        _enqueue(serial_line_buffer[i], true
#if NUM_SERIAL > 1
                 ,
                 i
#endif
        );
      }
      else if (serial_count[i] >= MAX_CMD_SIZE - 1)
      {
        // Keep fetching, but ignore normal characters beyond the max length
        // The command will be injected when EOL is reached
      }
      else if (serial_char == '\\')
      { // Handle escapes
        // if we have one more character, copy it over
        if ((c = read_serial(i)) >= 0 && !serial_comment_mode[i]
#if ENABLED(PAREN_COMMENTS)
            && !serial_comment_paren_mode[i]
#endif
        )
          serial_line_buffer[i][serial_count[i]++] = (char)c;
      }
      else
      { // it's not a newline, carriage return or escape char
        if (serial_char == ';')
          serial_comment_mode[i] = true;
#if ENABLED(PAREN_COMMENTS)
        else if (serial_char == '(')
          serial_comment_paren_mode[i] = true;
        else if (serial_char == ')')
          serial_comment_paren_mode[i] = false;
#endif
        else if (!serial_comment_mode[i]
#if ENABLED(PAREN_COMMENTS)
                 && !serial_comment_paren_mode[i]
#endif
        )
          serial_line_buffer[i][serial_count[i]++] = serial_char;
      }
    } // for NUM_SERIAL
  }   // queue has space, serial has data
}

#if ENABLED(SDSUPPORT)

/**
   * Get commands from the SD Card until the command buffer is full
   * or until the end of the file is reached. The special character '#'
   * can also interrupt buffering.
   */
inline void GCodeQueue::get_sdcard_commands()
{
  static bool stop_buffering = false,
              sd_comment_mode = false
#if ENABLED(PAREN_COMMENTS)
      ,
              sd_comment_paren_mode = false
#endif
      ;

  if (!IS_SD_PRINTING())
    return;

  /**
     * '#' stops reading from SD to the buffer prematurely, so procedural
     * macro calls are possible. If it occurs, stop_buffering is triggered
     * and the buffer is run dry; this character _can_ occur in serial com
     * due to checksums, however, no checksums are used in SD printing.
     */

  if (length == 0)
    stop_buffering = false;

  uint16_t sd_count = 0;
  bool card_eof = card.eof();
  while (length < BUFSIZE && !card_eof && !stop_buffering)
  {
    const int16_t n = card.get();
    char sd_char = (char)n;
    card_eof = card.eof();
    if (card_eof || n == -1 || sd_char == '\n' || sd_char == '\r' || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode
#if ENABLED(PAREN_COMMENTS)
                                                                      && !sd_comment_paren_mode
#endif
                                                                      ))
    {
      if (card_eof)
      {

        card.printingHasFinished();

        if (IS_SD_PRINTING())
          sd_count = 0; // If a sub-file was printing, continue from call point
        else
        {
          SERIAL_ECHOLNPGM(MSG_FILE_PRINTED);
#if ENABLED(PRINTER_EVENT_LEDS)
          printerEventLEDs.onPrintCompleted();
#if HAS_RESUME_CONTINUE
          inject_P(PSTR("M0 S"
#if HAS_LCD_MENU
                        "1800"
#else
                        "60"
#endif
                        ));
#endif
#endif // PRINTER_EVENT_LEDS
        }
      }
      else if (n == -1)
        SERIAL_ERROR_MSG(MSG_SD_ERR_READ);

      if (sd_char == '#')
        stop_buffering = true;

      sd_comment_mode = false; // for new command
#if ENABLED(PAREN_COMMENTS)
      sd_comment_paren_mode = false;
#endif

      // Skip empty lines and comments
      if (!sd_count)
      {
        thermalManager.manage_heater();
        continue;
      }

      command_buffer[index_w][sd_count] = '\0'; // terminate string
      sd_count = 0;                             // clear sd line buffer

      _commit_command(false);

#if ENABLED(POWER_LOSS_RECOVERY)
      recovery.cmd_sdpos = card.getIndex(); // Prime for the next _commit_command
#endif
    }
    else if (sd_count >= MAX_CMD_SIZE - 1)
    {
      /**
         * Keep fetching, but ignore normal characters beyond the max length
         * The command will be injected when EOL is reached
         */
    }
    else
    {
      if (sd_char == ';')
        sd_comment_mode = true;
#if ENABLED(PAREN_COMMENTS)
      else if (sd_char == '(')
        sd_comment_paren_mode = true;
      else if (sd_char == ')')
        sd_comment_paren_mode = false;
#endif
      else if (!sd_comment_mode
#if ENABLED(PAREN_COMMENTS)
               && !sd_comment_paren_mode
#endif
      )
        command_buffer[index_w][sd_count++] = sd_char;
    }
  }
}

#endif // SDSUPPORT

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void GCodeQueue::get_available_commands()
{

  get_serial_commands();

#if ENABLED(SDSUPPORT)
  get_sdcard_commands();
#endif
}

/**
 * Get the next command in the queue, optionally log it to SD, then dispatch it
 */
void GCodeQueue::advance()
{

  // Process immediate commands
  if (process_injected_command())
    return;

  // Return if the G-code buffer is empty
  if (!length)
    return;

#if ENABLED(SDSUPPORT)

  if (card.flag.saving)
  {
    char *command = command_buffer[index_r];
    if (is_M29(command))
    {
      // M29 closes the file
      card.closefile();
      SERIAL_ECHOLNPGM(MSG_FILE_SAVED);

#if !defined(__AVR__) || !defined(USBCON)
#if ENABLED(SERIAL_STATS_DROPPED_RX)
      SERIAL_ECHOLNPAIR("Dropped bytes: ", MYSERIAL0.dropped());
#endif

#if ENABLED(SERIAL_STATS_MAX_RX_QUEUED)
      SERIAL_ECHOLNPAIR("Max RX Queue Size: ", MYSERIAL0.rxMaxEnqueued());
#endif
#endif //  !defined(__AVR__) || !defined(USBCON)

      ok_to_send();
    }
    else
    {
      // Write the string from the read buffer to SD
      // card.write_command(command);
      // if (card.flag.logging)
      //   gcode.process_next_command(); // The card is saving because it's logging
      // else
      //   ok_to_send();
    }
  }
  else
    gcode.process_next_command();

#else

  gcode.process_next_command();

#endif // SDSUPPORT

  // The queue may be reset by a command handler or by code invoked by idle() within a handler
  if (length)
  {
    --length;
    if (++index_r >= BUFSIZE)
      index_r = 0;
  }
}
