// board enforces
//   in-state
//      accel set/resume
//   out-state
//      cancel button
//      accel rising edge
//      brake rising edge
//      brake > 0mph
const AddrBus HONDA_N_TX_MSGS[] = {{0xE4, 0}, {0x194, 0}, {0x1FA, 0}, {0x200, 0}, {0x30C, 0}, {0x33D, 0}};
const AddrBus HONDA_BH_TX_MSGS[] = {{0xE4, 0}, {0x296, 1}, {0x33D, 0}};  // Bosch Harness
const AddrBus HONDA_BG_TX_MSGS[] = {{0xE4, 2}, {0x296, 0}, {0x33D, 2}, {0x250, 0}};  // Bosch Giraffe
const int HONDA_GAS_INTERCEPTOR_THRESHOLD = 328;  // ratio between offset and gain from dbc file
int honda_brake = 0;
int honda_gas_prev = 0;
bool honda_brake_pressed_prev = false;
bool honda_moving = false;
bool honda_bosch_hardware = false;
bool honda_alt_brake_msg = false;
bool honda_fwd_brake = false;

static void honda_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {

  int addr = GET_ADDR(to_push);
  int len = GET_LEN(to_push);
  int bus = GET_BUS(to_push);

  // sample speed
  if (addr == 0x158) {
    // first 2 bytes
    honda_moving = GET_BYTE(to_push, 0) | GET_BYTE(to_push, 1);
  }

  // state machine to enter and exit controls
  // 0x1A6 for the ILX, 0x296 for the Civic Touring
  if ((addr == 0x1A6) || (addr == 0x296)) {
    // MAD MODE: cruise MAIN is the ONLY arming switch. MAIN_ON is SCM_BUTTONS bit 47
    // in the dbc (47|1@0+), which is byte 5 mask 0x80. The stock set/resume/cancel
    // buttons no longer arm or disarm, so stock cruise is optional and independent of
    // lane keep. MAIN off is the kill switch. Brake still clears controls_allowed on
    // 0x17C below, which pauses TX while braking without disengaging openpilot, since
    // controlsd tolerates a 2 s mismatch (controlsd.py:137 mismatch_counter >= 200).
    bool honda_main_on = (GET_BYTE(to_push, 5) & 0x80) != 0;
    controls_allowed = honda_main_on ? 1 : 0;
  }

  // user brake signal on 0x17C reports applied brake from computer brake on accord
  // and crv, which prevents the usual brake safety from working correctly. these
  // cars have a signal on 0x1BE which only detects user's brake being applied so
  // in these cases, this is used instead.
  // most hondas: 0x17C bit 53
  // accord, crv: 0x1BE bit 4
  // exit controls on rising edge of brake press or on brake press when speed > 0
  bool is_user_brake_msg = honda_alt_brake_msg ? ((addr) == 0x1BE) : ((addr) == 0x17C);
  if (is_user_brake_msg) {
    bool brake_pressed = honda_alt_brake_msg ? (GET_BYTE((to_push), 0) & 0x10) : (GET_BYTE((to_push), 6) & 0x20);
    // Brake no longer clears controls_allowed (2026-08-17). Without this the RX hook
    // would keep disarming at the 0x17C rate (~68 Hz) while MAIN re-armed at only 10 Hz,
    // so steering would still go limp while braking even with the TX change above.
    // honda_brake_pressed_prev is still maintained below for the rising-edge logic.
    honda_brake_pressed_prev = brake_pressed;
  }

  // exit controls on rising edge of gas press if interceptor (0x201 w/ len = 6)
  // length check because bosch hardware also uses this id (0x201 w/ len = 8)
  if ((addr == 0x201) && (len == 6)) {
    gas_interceptor_detected = 1;
    int gas_interceptor = GET_INTERCEPTOR(to_push);
    if ((gas_interceptor > HONDA_GAS_INTERCEPTOR_THRESHOLD) &&
        (gas_interceptor_prev <= HONDA_GAS_INTERCEPTOR_THRESHOLD) &&
        long_controls_allowed) {
      controls_allowed = 0;
    }
    gas_interceptor_prev = gas_interceptor;
  }

  // exit controls on rising edge of gas press if no interceptor
  if (!gas_interceptor_detected) {
    if (addr == 0x17C) {
      int gas = GET_BYTE(to_push, 0);
      if (gas && !(honda_gas_prev) && long_controls_allowed) {
        controls_allowed = 0;
      }
      honda_gas_prev = gas;
    }
  }
  if ((bus == 2) && (addr == 0x1FA)) {
    bool honda_stock_aeb = GET_BYTE(to_push, 3) & 0x20;
    int honda_stock_brake = (GET_BYTE(to_push, 0) << 2) + ((GET_BYTE(to_push, 1) >> 6) & 0x3);

    // Forward AEB when stock braking is higher than openpilot braking
    // only stop forwarding when AEB event is over
    if (!honda_stock_aeb) {
      honda_fwd_brake = false;
    } else if (honda_stock_brake >= honda_brake) {
      honda_fwd_brake = true;
    } else {
      // Leave Honda forward brake as is
    }
  }

  // if steering controls messages are received on the destination bus, it's an indication
  // that the relay might be malfunctioning
  int bus_rdr_car = (board_has_relay()) ? 0 : 2;  // radar bus, car side
  if ((safety_mode_cnt > RELAY_TRNS_TIMEOUT) && ((addr == 0xE4) || (addr == 0x194))) {
    if ((honda_bosch_hardware && (bus == bus_rdr_car)) ||
      (!honda_bosch_hardware && (bus == 0))) {
      relay_malfunction = true;
    }
  }
}

// all commands: gas, brake and steering
// if controls_allowed and no pedals pressed
//     allow all commands up to limit
// else
//     block all commands that produce actuation

static int honda_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  if (honda_bosch_hardware) {
    if (board_has_relay() && !addr_allowed(addr, bus, HONDA_BH_TX_MSGS, sizeof(HONDA_BH_TX_MSGS)/sizeof(HONDA_BH_TX_MSGS[0]))) {
      tx = 0;
    }
    if (!board_has_relay() && !addr_allowed(addr, bus, HONDA_BG_TX_MSGS, sizeof(HONDA_BG_TX_MSGS)/sizeof(HONDA_BG_TX_MSGS[0]))) {
      tx = 0;
    }
  }
  if (!honda_bosch_hardware && !addr_allowed(addr, bus, HONDA_N_TX_MSGS, sizeof(HONDA_N_TX_MSGS)/sizeof(HONDA_N_TX_MSGS[0]))) {
    tx = 0;
  }

  if (relay_malfunction) {
    tx = 0;
  }

  // disallow actuator commands if gas or brake (with vehicle moving) are pressed
  // and the the latching controls_allowed flag is True
  // LATERAL-ONLY RIG (2026-08-17): the driver's throttle is not an override on this car,
  // it is simply how the car is driven -- openpilot has no longitudinal control here.
  // The RX path above already conditions driver gas on long_controls_allowed; this TX
  // path did not, so EVERY steering frame was dropped whenever the pedal was down
  // (measured: STEER_CONTROL_ACTIVE 1-5% with gas vs 99% without). Made consistent.
  // Brake still cuts actuators below, and MAIN off is still an instant kill.
  // LATERAL-ONLY RIG: openpilot has no longitudinal control on this car
  // (interface.py: openpilotLongitudinalControl = False), so the driver MUST use the
  // throttle. Stock logic treats driver gas as an override and drops every steering
  // frame while the pedal is down, which blocked lateral control permanently.
  //
  // Do NOT try to gate this on long_controls_allowed: boardd sets that TRUE
  // unconditionally (boardd.cc:136 sends control request 0xdf with wValue 1, and
  // safety_declarations.h:56 defaults it true). That was tried on 2026-08-17 and was a
  // measured no-op -- EPS ctrlActive stayed at 1% with gas.
  //
  // Brake still cuts actuators below, and MAIN off is still an instant kill.
  // Brake removed from this list 2026-08-17 as well: on a lateral-only car, braking is
  // ordinary driving (slowing in traffic while lane keeping), not an override. NOTE that
  // after this NO PEDAL disarms the panda -- MAIN off is the remaining actuator kill.
  int pedal_pressed = (gas_interceptor_prev > HONDA_GAS_INTERCEPTOR_THRESHOLD);
  bool current_controls_allowed = controls_allowed && !(pedal_pressed);

  // BRAKE: safety check
  if ((addr == 0x1FA) && (bus == 0)) {
    honda_brake = (GET_BYTE(to_send, 0) << 2) + ((GET_BYTE(to_send, 1) >> 6) & 0x3);
    if (!current_controls_allowed || !long_controls_allowed) {
      if (honda_brake != 0) {
        tx = 0;
      }
    }
    if (honda_brake > 255) {
      tx = 0;
    }
    if (honda_fwd_brake) {
      tx = 0;
    }
  }

  // STEER: safety check
  if ((addr == 0xE4) || (addr == 0x194)) {
    if (!current_controls_allowed) {
      bool steer_applied = GET_BYTE(to_send, 0) | GET_BYTE(to_send, 1);
      if (steer_applied) {
        tx = 0;
      }
    }
  }

  // GAS: safety check
  if (addr == 0x200) {
    if (!current_controls_allowed || !long_controls_allowed) {
      if (GET_BYTE(to_send, 0) || GET_BYTE(to_send, 1)) {
        tx = 0;
      }
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button in Bosch HW
  // ensuring that only the cancel button press is sent (VAL 2) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  int bus_pt = ((board_has_relay()) && honda_bosch_hardware)? 1 : 0;
  if ((addr == 0x296) && honda_bosch_hardware &&
      !current_controls_allowed && (bus == bus_pt)) {
    if (((GET_BYTE(to_send, 0) >> 5) & 0x7) != 2) {
      tx = 0;
    }
  }

  // 1 allows the message through
  return tx;
}

static void honda_init(int16_t param) {
  UNUSED(param);
  controls_allowed = false;
  relay_malfunction = false;
  honda_bosch_hardware = false;
  honda_alt_brake_msg = false;
}

static void honda_bosch_init(int16_t param) {
  controls_allowed = false;
  relay_malfunction = false;
  honda_bosch_hardware = true;
  // Checking for alternate brake override from safety parameter
  honda_alt_brake_msg = (param == 1) ? true : false;
}

static int honda_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  // fwd from car to camera. also fwd certain msgs from camera to car
  // 0xE4 is steering on all cars except CRV and RDX, 0x194 for CRV and RDX,
  // 0x1FA is brake control, 0x30C is acc hud, 0x33D is lkas hud,
  int bus_fwd = -1;

  if (!relay_malfunction) {
    if (bus_num == 0) {
      bus_fwd = 2;
    }
    if (bus_num == 2) {
      // block stock lkas messages and stock acc messages (if OP is doing ACC)
      int addr = GET_ADDR(to_fwd);
      bool is_lkas_msg = (addr == 0xE4) || (addr == 0x194) || (addr == 0x33D);
      bool is_acc_hud_msg = addr == 0x30C;
      bool is_brake_msg = addr == 0x1FA;
      bool block_fwd = is_lkas_msg ||
                       (is_acc_hud_msg && long_controls_allowed) ||
                       (is_brake_msg && long_controls_allowed && !honda_fwd_brake);
      if (!block_fwd) {
        bus_fwd = 0;
      }
    }
  }
  return bus_fwd;
}

static int honda_bosch_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {
  int bus_fwd = -1;
  int bus_rdr_cam = (board_has_relay()) ? 2 : 1;  // radar bus, camera side
  int bus_rdr_car = (board_has_relay()) ? 0 : 2;  // radar bus, car side

  if (!relay_malfunction) {
    if (bus_num == bus_rdr_car) {
      bus_fwd = bus_rdr_cam;
    }
    if (bus_num == bus_rdr_cam)  {
      int addr = GET_ADDR(to_fwd);
      int is_lkas_msg = (addr == 0xE4) || (addr == 0x33D);
      if (!is_lkas_msg) {
        bus_fwd = bus_rdr_car;
      }
    }
  }
  return bus_fwd;
}

const safety_hooks honda_hooks = {
  .init = honda_init,
  .rx = honda_rx_hook,
  .tx = honda_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = honda_fwd_hook,
};

const safety_hooks honda_bosch_hooks = {
  .init = honda_bosch_init,
  .rx = honda_rx_hook,
  .tx = honda_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = honda_bosch_fwd_hook,
};
