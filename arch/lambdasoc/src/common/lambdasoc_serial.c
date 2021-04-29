/****************************************************************************
 * arch/lambdasoc/src/common/lambdasoc_serial.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "chip.h"
#include "hw/uart.h"
#include "lambdasoc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_UART_DEVICE
#if defined(CONFIG_LAMBDASOC_UART1) || defined(CONFIG_LAMBDASOC_UART2)
#  define HAVE_UART_DEVICE 1
#endif

/* Is there a serial console?  There should be no more than one defined.  It
 * could be on any UARTn, n=1,.. CHIP_NUARTS
 */

#if defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_LAMBDASOC_UART1)
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#elif defined(CONFIG_UART2_SERIAL_CONSOLE) && defined(CONFIG_LAMBDASOC_UART2)
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  define HAVE_SERIAL_CONSOLE 1
#else
#  undef CONFIG_UART1_SERIAL_CONSOLE
#  undef CONFIG_UART2_SERIAL_CONSOLE
#  undef HAVE_SERIAL_CONSOLE
#endif

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of lambdasoc_putc.
 */

#ifdef USE_SERIALDRIVER

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define CONSOLE_BASE    LAMBDASOC_UART1_BASE
#    define CONSOLE_DEV     g_uart1port     /* UART1 is console */
#    define TTYS0_DEV       g_uart1port     /* UART1 is ttyS0 */
#    undef  TTYS1_DEV                       /* No ttyS1 */
#    define SERIAL_CONSOLE  1
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_BASE
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART1_SERIAL_CONSOLE
#  if defined(CONFIG_NR5_UART1)
#    define TTYS0_DEV       g_uart1port     /* UART1 is ttyS0 */
#    undef  TTYS1_DEV                       /* No ttyS1 */
#    define SERIAL_CONSOLE  1
#  else
#    undef  TTYS0_DEV
#    undef  TTYS1_DEV
#  endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of lambdasoc_earlyserialinit(),
 * lambdasoc_serial_initialize(), and lambdasoc_putc().
 */

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lambdasoc_dev_s
{
  uintptr_t uartbase;
  uintptr_t divisor_addr;
  uintptr_t rx_data_addr;
  uintptr_t rx_rdy_addr;
  uintptr_t rx_err_addr;
  uintptr_t tx_data_addr;
  uintptr_t tx_rdy_addr;
  uintptr_t ev_status_addr;
  uintptr_t ev_pending_addr;
  uintptr_t ev_enable_addr;
  uint8_t irq;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers */

static void lambdasoc_restoreuartint(struct uart_dev_s *dev, uint32_t im);
static void lambdasoc_disableuartint(struct uart_dev_s *dev, uint32_t *im);

/* Serial driver methods */

static int  lambdasoc_setup(struct uart_dev_s *dev);
static void lambdasoc_shutdown(struct uart_dev_s *dev);
static int  lambdasoc_attach(struct uart_dev_s *dev);
static void lambdasoc_detach(struct uart_dev_s *dev);
static int  lambdasoc_uart_interrupt(int irq, void *context, FAR void *arg);
static int  lambdasoc_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  lambdasoc_receive(struct uart_dev_s *dev, uint32_t *status);
static void lambdasoc_rxint(struct uart_dev_s *dev, bool enable);
static bool lambdasoc_rxavailable(struct uart_dev_s *dev);
static void lambdasoc_send(struct uart_dev_s *dev, int ch);
static void lambdasoc_txint(struct uart_dev_s *dev, bool enable);
static bool lambdasoc_txready(struct uart_dev_s *dev);
static bool lambdasoc_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = lambdasoc_setup,
  .shutdown       = lambdasoc_shutdown,
  .attach         = lambdasoc_attach,
  .detach         = lambdasoc_detach,
  .ioctl          = lambdasoc_ioctl,
  .receive        = lambdasoc_receive,
  .rxint          = lambdasoc_rxint,
  .rxavailable    = lambdasoc_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = lambdasoc_send,
  .txint          = lambdasoc_txint,
  .txready        = lambdasoc_txready,
  .txempty        = lambdasoc_txempty,
};

/* I/O buffers */

#ifdef CONFIG_LAMBDASOC_UART1
static char g_uart1rxbuffer[LAMBDASOC_UART1_RXBUFSIZE];
static char g_uart1txbuffer[LAMBDASOC_UART1_TXBUFSIZE];
#endif

/* This describes the state of the NR5 UART1 port. */

#ifdef CONFIG_LAMBDASOC_UART1
#ifndef CONFIG_LAMBDASOC_UART1PRIO
# define CONFIG_LAMBDASOC_UART1PRIO 4
#endif

static struct lambdasoc_dev_s g_uart1priv =
{
  .uartbase        = LAMBDASOC_UART1_BASE,
  .irq             = LAMBDASOC_UART1_IRQNO,
  .divisor_addr    = LAMBDASOC_UART1_DIVISOR,
  .rx_data_addr    = LAMBDASOC_UART1_RX_DATA,
  .rx_rdy_addr     = LAMBDASOC_UART1_RX_RDY,
  .rx_err_addr     = LAMBDASOC_UART1_RX_ERR,
  .tx_data_addr    = LAMBDASOC_UART1_TX_DATA,
  .tx_rdy_addr     = LAMBDASOC_UART1_TX_RDY,
  .ev_status_addr  = LAMBDASOC_UART1_EV_STATUS,
  .ev_pending_addr = LAMBDASOC_UART1_EV_PENDING,
  .ev_enable_addr  = LAMBDASOC_UART1_EV_ENABLE,
};

static uart_dev_t g_uart1port =
{
#if SERIAL_CONSOLE == 1
  .isconsole = 1,
#endif
  .recv      =
  {
    .size    = LAMBDASOC_UART1_RXBUFSIZE,
    .buffer  = g_uart1rxbuffer,
  },
  .xmit      =
  {
    .size    = LAMBDASOC_UART1_TXBUFSIZE,
    .buffer  = g_uart1txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart1priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lambdasoc_restoreuartint
 ****************************************************************************/

static void lambdasoc_restoreuartint(struct uart_dev_s *dev, uint32_t im)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;

  /* Re-enable/re-disable interrupts corresponding to the state of bits in
   * im.
   */

  putreg32(im, priv->ev_enable_addr);
}

/****************************************************************************
 * Name: lambdasoc_disableuartint
 ****************************************************************************/

static void lambdasoc_disableuartint(struct uart_dev_s *dev, uint32_t *im)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;

  if (im)
    {
      *im = getreg32(priv->ev_enable_addr);
    }

  lambdasoc_restoreuartint(dev, 0);
}

/****************************************************************************
 * Name: lambdasoc_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int lambdasoc_setup(struct uart_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;

  uint32_t pending = getreg32(priv->ev_pending_addr);
  putreg32(pending, priv->ev_pending_addr);

  return OK;
}

/****************************************************************************
 * Name: lambdasoc_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void lambdasoc_shutdown(struct uart_dev_s *dev)
{
}

/****************************************************************************
 * Name: lambdasoc_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() methods are
 *   called.
 *
 ****************************************************************************/

static int lambdasoc_attach(struct uart_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;

  irq_attach(priv->irq, lambdasoc_uart_interrupt, dev);
  up_enable_irq(priv->irq);

  return OK;
}

/****************************************************************************
 * Name: lambdasoc_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.
 *   The exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void lambdasoc_detach(struct uart_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: lambdasoc_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the 'irq'  It should call uart_transmitchars or
 *   uart_receivechar to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'irq' number into the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int lambdasoc_uart_interrupt(int irq, void *context, FAR void *arg)
{
  DEBUGASSERT(arg != NULL);
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;

  uint32_t stat;

  /* Read as much as we can */

  stat = getreg32(priv->ev_pending_addr);
  stat &= getreg32(priv->ev_enable_addr);

  if (stat & UART_EV_RX_RDY)
    {
      while (getreg32(priv->rx_rdy_addr))
        {
          uart_recvchars(dev);
        }
    }

  /* Try to send all the buffer that were not sent.  Does uart_xmitchars
   * send only if there is something to send ???
   */

  if ((stat & UART_EV_TX_MTY) != 0)
    {
      putreg32(UART_EV_TX_MTY, priv->ev_pending_addr);
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: lambdasoc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int lambdasoc_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#ifdef CONFIG_SERIAL_TERMIOS
  return -ENOSYS;
#else
  return -ENOTTY;
#endif
}

/****************************************************************************
 * Name: lambdasoc_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int lambdasoc_receive(struct uart_dev_s *dev, uint32_t *status)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;
  int ret;

  if (status != NULL)
    {
      *status = 0; // TODO read from UART_RX_ERR_OFFSET
    }

  ret = getreg32(priv->rx_data_addr);

  return ret;
}

/****************************************************************************
 * Name: lambdasoc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void lambdasoc_rxint(struct uart_dev_s *dev, bool enable)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;
  uint32_t im;

  im = getreg32(priv->ev_enable_addr);
  if (enable)
    {
      im |= UART_EV_RX_RDY;
      im |= UART_EV_RX_ERR;
    }
  else
    {
      im &= ~UART_EV_RX_RDY;
      im &= ~UART_EV_RX_ERR;
    }

  putreg32(im, priv->ev_enable_addr);
}

/****************************************************************************
 * Name: lambdasoc_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool lambdasoc_rxavailable(struct uart_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;
  return !!getreg32(priv->rx_rdy_addr);
}

/****************************************************************************
 * Name: lambdasoc_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void lambdasoc_send(struct uart_dev_s *dev, int ch)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;
  putreg32(ch, priv->tx_data_addr);
}

/****************************************************************************
 * Name: lambdasoc_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void lambdasoc_txint(struct uart_dev_s *dev, bool enable)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;
  uint32_t im;

  im = getreg32(priv->ev_enable_addr);
  if (enable)
    {
        im |= UART_EV_TX_MTY;
        putreg32(im, priv->ev_enable_addr);

        /* Fake an uart INT */

        uart_xmitchars(dev);
    }
  else
    {
        im &= ~UART_EV_TX_MTY;
        putreg32(im, priv->ev_enable_addr);
    }
}

/****************************************************************************
 * Name: lambdasoc_txready
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool lambdasoc_txready(struct uart_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;
  return !!getreg32(priv->tx_rdy_addr);
}

/****************************************************************************
 * Name: lambdasoc_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool lambdasoc_txempty(struct uart_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  struct lambdasoc_dev_s *priv = (struct lambdasoc_dev_s *)dev->priv;
  uint32_t ev_status = getreg32(priv->ev_status_addr);
  return !!(ev_status & UART_EV_TX_MTY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lambdasoc_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before lambdasoc_serial_initialize.
 *
 ****************************************************************************/

void lambdasoc_earlyserialinit(void)
{
}

/****************************************************************************
 * Name: lambdasoc_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  struct uart_dev_s *dev = (struct uart_dev_s *)&CONSOLE_DEV;
  uint32_t imr;

  lambdasoc_disableuartint(dev, &imr);

  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      lambdasoc_lowputc('\r');
    }

  lambdasoc_lowputc(ch);
  lambdasoc_restoreuartint(dev, imr);
#endif
  return ch;
}

/****************************************************************************
 * Name: lambdasoc_earlyserialinit, lambdasoc_serial_initialize, and lambdasoc_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/lambdasoc_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

#else /* HAVE_UART_DEVICE */
void lambdasoc_earlyserialinit(void)
{
}

void lambdasoc_serial_initialize(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: lambdasoc_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      lambdasoc_lowputc('\r');
    }

  lambdasoc_lowputc(ch);
#endif

  return ch;
}

#endif /* USE_SERIALDRIVER */

void lambdasoc_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the TX FIFO is non-full */

  while (!getreg32(LAMBDASOC_UART1_TX_RDY))
    ;

  /* Then send the character */
  putreg32(ch, LAMBDASOC_UART1_TX_DATA);
  putreg32(UART_EV_TX_MTY, LAMBDASOC_UART1_EV_PENDING);
#endif
}

/****************************************************************************
 * Name: lambdasoc_serial_initialize
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that lambdasoc_earlyserialinit was called previously.
 *
 ****************************************************************************/

void lambdasoc_serial_initialize(void)
{
#ifdef USE_SERIALDRIVER
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
}
