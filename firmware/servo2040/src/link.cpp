#include "link.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/sync.h"
#include "protocol/protocol.h"

namespace link
{

static uart_inst_t *const UART = uart1;
constexpr uint TX_PIN = 20;
constexpr uint RX_PIN = 21;
constexpr uint BAUD = 115200;

// Receive ring, filled by the interrupt and drained by read(). Several ticks
// of command frames fit, so a late tick loses nothing.
constexpr uint32_t RX_SIZE = RX_BUFFER_SIZE; // power of two
static volatile uint8_t rxBuf[RX_SIZE];
static volatile uint32_t rxHead; // written by the interrupt
static volatile uint32_t rxTail; // written by read()
static volatile uint32_t rxDropped;

// Transmit queue of whole frames, fed to the FIFO by the interrupt. Holds a
// calibration report and a telemetry frame at once.
constexpr uint32_t TX_QUEUE_SIZE = 256; // power of two
static_assert(TX_QUEUE_SIZE >= protocol::CALIBRATION_REPORT_FRAME_SIZE + protocol::TELEMETRY_FRAME_SIZE,
              "a report and a telemetry frame fit together");
static volatile uint8_t txBuf[TX_QUEUE_SIZE];
static volatile uint32_t txHead; // written by queueFrame()
static volatile uint32_t txTail; // written by the interrupt (and queueFrame() with it masked)

static void fillTxFifo()
{
  uint32_t tail = txTail;
  while (tail != txHead && uart_is_writable(UART))
    uart_get_hw(UART)->dr = txBuf[tail++ % TX_QUEUE_SIZE];
  txTail = tail;
  if (tail == txHead)
    uart_set_irqs_enabled(UART, true, false); // nothing left: stop TX interrupts
}

static void onUartIrq()
{
  while (uart_is_readable(UART))
  {
    uint32_t dr = uart_get_hw(UART)->dr;
    // Framing, parity, break or overrun: the byte is unreliable.
    if (dr & (UART_UARTDR_FE_BITS | UART_UARTDR_PE_BITS | UART_UARTDR_BE_BITS | UART_UARTDR_OE_BITS))
    {
      rxDropped = rxDropped + 1;
      continue;
    }
    uint32_t head = rxHead;
    if (head - rxTail >= RX_SIZE)
    {
      rxDropped = rxDropped + 1; // read() fell behind
      continue;
    }
    rxBuf[head % RX_SIZE] = (uint8_t)dr;
    rxHead = head + 1;
  }
  if (txTail != txHead)
    fillTxFifo();
}

void setup()
{
  uart_init(UART, BAUD);
  uart_set_format(UART, 8, 1, UART_PARITY_NONE);
  uart_set_hw_flow(UART, false, false);
  uart_set_fifo_enabled(UART, true);
  gpio_set_function(TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(RX_PIN, GPIO_FUNC_UART);
  // An unplugged bridge leaves RX idling high rather than floating.
  gpio_pull_up(RX_PIN);

  irq_set_exclusive_handler(UART_IRQ_NUM(UART), onUartIrq);
  irq_set_enabled(UART_IRQ_NUM(UART), true);
  uart_set_irqs_enabled(UART, true, false);
}

int read(uint8_t *out, int max)
{
  // Bytes that arrive meanwhile wait for the next tick.
  uint32_t head = rxHead;
  int n = 0;
  while (rxTail != head && n < max)
  {
    out[n++] = rxBuf[rxTail % RX_SIZE];
    rxTail = rxTail + 1;
  }
  return n;
}

static bool queueFrame(const uint8_t *frame, int n)
{
  irq_set_enabled(UART_IRQ_NUM(UART), false);
  uint32_t head = txHead;
  bool fits = TX_QUEUE_SIZE - (head - txTail) >= (uint32_t)n;
  if (fits)
  {
    for (int i = 0; i < n; i++)
      txBuf[head++ % TX_QUEUE_SIZE] = frame[i];
    txHead = head;
    // Prime the FIFO here; the TX interrupt (raised as the FIFO drains past
    // its threshold) sends the rest.
    fillTxFifo();
    if (txTail != txHead)
      uart_set_irqs_enabled(UART, true, true);
  }
  irq_set_enabled(UART_IRQ_NUM(UART), true);
  return fits;
}

bool sendFrame(const uint8_t *frame, int length)
{
  return queueFrame(frame, length);
}

uint32_t droppedBytes() { return rxDropped; }

} // namespace link
