#include "StepperISR.h"
#if defined(HAVE_ESP32_RMT) && (ESP_IDF_VERSION_MAJOR == 4)

// #define TEST_MODE

#include "fas_arch/test_probe.h"

// The following concept is in use:
//
//    The buffer of 64Bytes is split into two parts à 31 words.
//    Each part will hold one command (or part of).
//    After the 2*31 words an end marker is placed.
//    The threshold is set to 31.
//
//    This way, the threshold interrupt occurs after the first part
//    and the end interrupt after the second part.
//
// Of these 32 bits, the low 16-bit entry is sent first and the high entry
// second.
// Every 16 bit entry defines with MSB the output level and the lower 15 bits
// the ticks.

void IRAM_ATTR StepperQueue::stop_rmt(bool both) {
  // We are stopping the rmt by letting it run into the end at high speed.
  //
  // disable the interrupts
  //  rmt_set_tx_intr_en(channel, false);
  //  rmt_set_tx_thr_intr_en(channel, false, 0);

  // stop esp32 rmt, by let it hit the end
  RMT.conf_ch[channel].conf1.tx_conti_mode = 0;

  // replace second part of buffer with pauses
  uint32_t *data = FAS_RMT_MEM(channel);
  uint8_t start = both ? 0 : PART_SIZE;
  data = &data[start];
  for (uint8_t i = start; i < 2 * PART_SIZE; i++) {
    // two pauses à n ticks to achieve MIN_CMD_TICKS
    *data++ = 0x00010001 * ((MIN_CMD_TICKS + 61) / 62);
  }
  *data = 0;

  // as the rmt is not running anymore, mark it as stopped
  _rmtStopped = true;
}

#ifndef RMT_CHANNEL_MEM
#define RMT_LIMIT tx_lim
#define RMT_FIFO apb_fifo_mask
#else
#define RMT_LIMIT limit
#define RMT_FIFO fifo_mask
#endif

#ifdef TEST_MODE
#define PROCESS_CHANNEL(ch)                                    \
  if (mask & RMT_CH##ch##_TX_END_INT_ST) {                     \
    PROBE_2_TOGGLE;                                            \
  }                                                            \
  if (mask & RMT_CH##ch##_TX_THR_EVENT_INT_ST) {               \
    PROBE_3_TOGGLE;                                            \
    /* now repeat the interrupt at buffer size + end marker */ \
    RMT.tx_lim_ch[ch].RMT_LIMIT = PART_SIZE * 2 + 1;           \
  }
#else
#define PROCESS_CHANNEL(ch)                                      \
  if (mask & RMT_CH##ch##_TX_END_INT_ST) {                       \
    PROBE_2_TOGGLE;                                              \
    StepperQueue *q = &fas_queue[QUEUES_MCPWM_PCNT + ch];        \
    if (q->_rmtStopped) {                                        \
      rmt_set_tx_intr_en(q->channel, false);                     \
      rmt_set_tx_thr_intr_en(q->channel, false, PART_SIZE + 1);  \
      q->_isRunning = false;                                     \
      PROBE_1_TOGGLE;                                            \
    } else {                                                     \
      rmt_apply_command(q, false, FAS_RMT_MEM(ch));              \
    }                                                            \
  }                                                              \
  if (mask & RMT_CH##ch##_TX_THR_EVENT_INT_ST) {                 \
    PROBE_3_TOGGLE;                                              \
    StepperQueue *q = &fas_queue[QUEUES_MCPWM_PCNT + ch];        \
    if (!q->_rmtStopped) {                                       \
      rmt_apply_command(q, true, FAS_RMT_MEM(ch));               \
      /* now repeat the interrupt at buffer size + end marker */ \
      RMT.tx_lim_ch[ch].RMT_LIMIT = PART_SIZE * 2 + 1;           \
    }                                                            \
  }
#endif

static void IRAM_ATTR tx_intr_handler(void *arg) {
  uint32_t mask = RMT.int_st.val;
  RMT.int_clr.val = mask;
  PROCESS_CHANNEL(0);
  PROCESS_CHANNEL(1);
#if QUEUES_RMT >= 4
  PROCESS_CHANNEL(2);
  PROCESS_CHANNEL(3);
#endif
#if QUEUES_RMT == 8
  PROCESS_CHANNEL(4);
  PROCESS_CHANNEL(5);
  PROCESS_CHANNEL(6);
  PROCESS_CHANNEL(7);
#endif
}

bool StepperQueue::init_rmt(uint8_t channel_num, uint8_t step_pin) {
#ifdef TEST_PROBE_1
  if (channel_num == 0) {
    pinMode(TEST_PROBE_1, OUTPUT);
    PROBE_1(HIGH);
    delay(10);
    PROBE_1(LOW);
    delay(5);
    PROBE_1(HIGH);
    delay(5);
    PROBE_1(LOW);
    delay(5);
  }
#endif
#ifdef TEST_PROBE_2
  pinMode(TEST_PROBE_2, OUTPUT);
  PROBE_2(LOW);
#endif
#ifdef TEST_PROBE_3
  pinMode(TEST_PROBE_3, OUTPUT);
  PROBE_3(LOW);
#endif

  _initVars();
  _step_pin = step_pin;
  digitalWrite(step_pin, LOW);
  pinMode(step_pin, OUTPUT);

  channel = (rmt_channel_t)channel_num;

  if (channel_num == 0) {
    periph_module_enable(PERIPH_RMT_MODULE);
  }
  // 80 MHz/5 = 16 MHz
  rmt_set_tx_intr_en(channel, false);
  rmt_set_tx_thr_intr_en(channel, false, PART_SIZE + 1);
  rmt_set_source_clk(channel, RMT_BASECLK_APB);
  rmt_set_clk_div(channel, 5);
  rmt_set_mem_block_num(channel, 1);
  rmt_set_tx_carrier(channel, false, 0, 0, RMT_CARRIER_LEVEL_LOW);
  rmt_tx_stop(channel);
  rmt_rx_stop(channel);
  // rmt_tx_memory_reset is not defined in arduino V340 and based on test result
  // not needed rmt_tx_memory_reset(channel);
  if (channel_num == 0) {
    rmt_isr_register(tx_intr_handler, NULL,
                     ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM, NULL);
    RMT.apb_conf.RMT_FIFO = 1;  // disable fifo mode
    RMT.apb_conf.mem_tx_wrap_en = 0;
  }

  _isRunning = false;
  _rmtStopped = true;

  connect();

#ifdef TEST_MODE
  if (channel == 0) {
    uint32_t *mem = FAS_RMT_MEM(channel);
    // Fill the buffer with a significant pattern for debugging
    for (uint8_t i = 0; i < 32; i += 2) {
      *mem++ = 0x7fffffff;
      *mem++ = 0x7fff7fff;
    }
    for (uint8_t i = 32; i < 62; i += 2) {
      *mem++ = 0x3fffdfff;
      *mem++ = 0x3fff3fff;
    }
    // without end marker it does not loop
    *mem++ = 0;
    *mem++ = 0;
    RMT.conf_ch[channel].conf1.tx_conti_mode = 1;
    rmt_set_tx_intr_en(channel, true);
    rmt_set_tx_thr_intr_en(channel, true, PART_SIZE + 1);
    // rmt_tx_start(channel, true);
    PROBE_2_TOGGLE;

    delay(1000);
    if (false) {
      mem--;
      mem--;
      // destroy end marker => no end interrupt, no repeat
      *mem++ = 0x00010001;
      *mem = 0x00010001;
    }
    if (true) {
      // just clear conti mode => causes end interrup, no repeat
      RMT.conf_ch[channel].conf1.tx_conti_mode = 0;
    }
    delay(1000);
    // actually no need to enable/disable interrupts.
    // and this seems to avoid some pitfalls

    // This runs the RMT buffer once
    RMT.conf_ch[channel].conf1.tx_conti_mode = 1;
    delay(1);
    RMT.conf_ch[channel].conf1.tx_conti_mode = 0;
    while (true) {
      delay(1);
    }
  }
#endif
  return true;
}

void StepperQueue::connect_rmt() {
  // rmt_set_tx_intr_en(channel, true);
  // rmt_set_tx_thr_intr_en(channel, true, PART_SIZE + 1);
  RMT.conf_ch[channel].conf1.idle_out_lv = 0;
  RMT.conf_ch[channel].conf1.idle_out_en = 1;
#ifndef __ESP32_IDF_V44__
  rmt_set_pin(channel, RMT_MODE_TX, (gpio_num_t)_step_pin);
#else
  rmt_set_gpio(channel, RMT_MODE_TX, (gpio_num_t)_step_pin, false);
#endif
}

void StepperQueue::disconnect_rmt() {
  rmt_set_tx_intr_en(channel, false);
  rmt_set_tx_thr_intr_en(channel, false, PART_SIZE + 1);
#ifndef __ESP32_IDF_V44__
//  rmt_set_pin(channel, RMT_MODE_TX, (gpio_num_t)-1);
#else
  rmt_set_gpio(channel, RMT_MODE_TX, GPIO_NUM_NC, false);
#endif
  RMT.conf_ch[channel].conf1.idle_out_en = 0;
}

void StepperQueue::startQueue_rmt() {
// #define TRACE
#ifdef TRACE
  Serial.println("START");
#endif
#ifdef TEST_PROBE_2
  PROBE_2(LOW);
#endif
#ifdef TEST_PROBE_3
  PROBE_3(LOW);
#endif
#ifdef TEST_PROBE_1
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
  PROBE_1_TOGGLE;
  delay(1);
#endif
  rmt_tx_stop(channel);
  // rmt_rx_stop(channel);
  // rmt_tx_memory_reset(channel);
  // the following assignment should not be needed;
  // RMT.data_ch[channel] = 0;
  uint32_t *mem = FAS_RMT_MEM(channel);
  // Fill the buffer with a significant pattern for debugging
  // Keep it for now
  for (uint8_t i = 0; i < 2 * PART_SIZE; i += 2) {
    mem[i] = 0x0fff8fff;
    mem[i + 1] = 0x7fff8fff;
  }
  mem[2 * PART_SIZE] = 0;
  mem[2 * PART_SIZE + 1] = 0;
  _isRunning = true;
  _rmtStopped = false;
  rmt_set_tx_intr_en(channel, false);
  rmt_set_tx_thr_intr_en(channel, false, 0);
  // RMT.apb_conf.mem_tx_wrap_en = 0;

// #define TRACE
#ifdef TRACE
  Serial.print("Queue:");
  Serial.print(read_idx);
  Serial.print('/');
  Serial.println(next_write_idx);
  for (uint8_t i = 0; i < 64; i++) {
    Serial.print(i);
    Serial.print(' ');
    Serial.println(mem[i], HEX);
  }
#endif

  // set dirpin toggle here
  uint8_t rp = read_idx;
  if (rp == next_write_idx) {
    // nothing to do ?
    // Should not happen, so bail
    return;
  }
  if (entry[rp & QUEUE_LEN_MASK].toggle_dir) {
    LL_TOGGLE_PIN(dirPin);
    entry[rp & QUEUE_LEN_MASK].toggle_dir = false;
  }

  lastChunkContainsSteps = true;
  rmt_apply_command(this, true, mem);

#ifdef TRACE
  Serial.print(RMT.conf_ch[channel].conf0.val, BIN);
  Serial.print(' ');
  Serial.print(RMT.conf_ch[channel].conf1.val, BIN);
  Serial.println(' ');
  Serial.print(RMT.apb_conf.mem_tx_wrap_en);
  Serial.println(' ');
  for (uint8_t i = 0; i < 64; i++) {
    Serial.print(i);
    Serial.print(' ');
    Serial.println(mem[i], HEX);
  }
  if (!isRunning()) {
    Serial.println("STOPPED 1");
  }
#endif

  rmt_apply_command(this, false, mem);

#ifdef TRACE
  Serial.print(RMT.conf_ch[channel].conf0.val, BIN);
  Serial.print(' ');
  Serial.print(RMT.conf_ch[channel].conf1.val, BIN);
  Serial.println(' ');
  Serial.print(RMT.apb_conf.mem_tx_wrap_en);
  Serial.println(' ');

  for (uint8_t i = 0; i < 64; i++) {
    Serial.print(i);
    Serial.print(' ');
    Serial.println(mem[i], HEX);
  }
#endif
  rmt_set_tx_thr_intr_en(channel, true, PART_SIZE + 1);
  rmt_set_tx_intr_en(channel, true);
  _rmtStopped = false;

  // This starts the rmt module
  RMT.conf_ch[channel].conf1.tx_conti_mode = 1;

  RMT.conf_ch[channel].conf1.tx_start = 1;
}
void StepperQueue::forceStop_rmt() {
  stop_rmt(true);

  // and empty the buffer
  read_idx = next_write_idx;
}
bool StepperQueue::isReadyForCommands_rmt() {
  if (_isRunning) {
    return !_rmtStopped;
  }
  return true;
}
uint16_t StepperQueue::_getPerformedPulses_rmt() { return 0; }
#endif
