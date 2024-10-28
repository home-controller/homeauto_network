/**
 * @file message_ids.h
 * @author Joseph (you@domain.com)
 * @brief
 * @version 0.1.0
 * @date 2024-09-1
 *
 * @copyright Copyright (c) 2024
 *
 */

/// This is copied from
/// https://github.com/home-controller/home_auto/blob/master/lights_controller/src/commands.h
/// This could all be consolidated into a .h file that can be included into
/// other home auto units. maybe with other #defines for rooms, groups etc.

#ifndef _message_ids_h
#define _message_ids_h

#define c_node \
  B1100  // command to a node with specified id. Changed from b0000 to b1000 to
         // give lower priority
#define c_lightAct \
  B1001                  // command to act on a specific light. given by room and light no. in
                         // room. eg. turn it off.
#define c_lightOn B0100  // command to turn on a light.
#define c_extended \
  B0011                       // as b0001 but used when room id or light id > 16.  rooms and lights
                              // per can be up to 256 instead of 16 {expect 20 more bits}
#define mId_sw_comms B1011    /// Message Id to send for when a switch has been switched.
#define c_command_list B1010  // command list No.
#define c_list8 B0010         // command list No.

#define c_group B1110  // bit B01000000 = group command

// 4 bits for command type + 4 bits for action/value
// actions. Used with command type.
#define c_turn_off 0            // ct stands for command type
#define c_turn_on 1             // 001
#define c_toggle 2              // 010
#define c_set_dimmer B100       // 4    note. dimmer is a 5 bit value a lot of places
#define c_set_dimmerNight B101  // 5: set dimmer at 12%
/*
 * maybe?  c_set_dimmer  B1001  //15
  #define c_set_dimmer  B1010  //30%
  #define c_set_dimmer  B1011  //40%
  #define c_set_dimmer  B1100  //50%
  #define c_set_dimmer  B1101  //60%
  #define c_set_dimmer  B1110  //70%
  #define c_set_dimmer  B1111  //80%
*/
// Temp
#define c_temp_high_trip B101  // = 5
#define c_temp_low_alarm B110  // 6
#define c_temp_set_high B111   // = 5
#define c_temp_set_low B1000   // 6
#define c_aMask B11111

#endif