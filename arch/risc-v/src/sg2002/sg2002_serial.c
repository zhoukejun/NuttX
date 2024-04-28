/****************************************************************************
 * arch/risc-v/src/sg2002/sg2002_serial.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <math.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/serial/tioctl.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "sg2002_config.h"
#include "chip.h"
#include "sg2002.h"

#include "hardware/sg2002_memorymap.h"
#include "hardware/sg2002_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If we are not using the serial driver for the console, then we still must
 * provide some minimal implementation of up_putc.
 */

#ifdef USE_SERIALDRIVER

#define CONFIG_UART_SCLK	25000000

/* Which UART with be tty0/console and which tty1?  The console will always
 * be ttyS0.  If there is no console then will use the lowest numbered UART.
 */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define CONSOLE_DEV     g_uart0port     /* UART0 is console */
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    define SERIAL_CONSOLE  1
#  else
#    error "I'm confused... Do we have a serial console or not?"
#  endif
#else
#  undef  CONSOLE_DEV                        /* No console */
#  undef  CONFIG_UART0_SERIAL_CONSOLE
#  if defined(CONFIG_SG2002_UART0)
#    define TTYS0_DEV       g_uart0port     /* UART0 is ttyS0 */
#    define SERIAL_CONSOLE  1
#  else
#    undef  TTYS0_DEV
#  endif
#endif

/* Common initialization logic will not not know that the all of the UARTs
 * have been disabled.  So, as a result, we may still have to provide
 * stub implementations of riscv_earlyserialinit(), riscv_serialinit(), and
 * up_putc().
 */

#ifdef HAVE_UART_DEVICE

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct up_dev_s
{
  uintptr_t uartbase; /* Base address of UART registers */
  uint32_t  baud;     /* Configured baud */
  uint8_t   irq;      /* IRQ associated with this UART */
  uint8_t   im;       /* Interrupt mask state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers */

static uint32_t up_serialin(struct up_dev_s *priv, int offset);
static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value);
static void up_restoreuartint(struct up_dev_s *priv, uint8_t im);
static void up_disableuartint(struct up_dev_s *priv, uint8_t *im);

/* Serial driver methods */

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, uint32_t *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};

/* I/O buffers */

#ifdef CONFIG_SG2002_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];
#endif

#ifdef CONFIG_SG2002_UART0
static struct up_dev_s g_uart0priv =
{
  .uartbase  = SG2002_UART0_BASE,
  .baud      = CONFIG_UART0_BAUD,
  .irq       = SG2002_IRQ_UART0,
};

static uart_dev_t g_uart0port =
{
#if SERIAL_CONSOLE == 1
  .isconsole = 1,
#endif
  .recv      =
  {
    .size    = CONFIG_UART0_RXBUFSIZE,
    .buffer  = g_uart0rxbuffer,
  },
  .xmit      =
  {
    .size    = CONFIG_UART0_TXBUFSIZE,
    .buffer  = g_uart0txbuffer,
  },
  .ops       = &g_uart_ops,
  .priv      = &g_uart0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void sg2002_putc(struct up_dev_s *priv, uint8_t ch)
{
#ifdef HAVE_SERIAL_CONSOLE

  uint8_t imr;
  up_disableuartint(priv, &imr);
  /* Wait until the TX data register is empty */

  while (!(up_serialin(priv, UART_LSR) & LSR_TRANS_HOLDING_REG_EMPTY))
    {
    }

  /* Then send the character */

  up_serialout(priv, UART_RBR_THR_DLL, ch);

  up_restoreuartint(priv, imr);
#endif /* HAVE_CONSOLE */
}


/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static void up_serialout(struct up_dev_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static void up_restoreuartint(struct up_dev_s *priv, uint8_t im)
{
  priv->im = im;

  up_serialout(priv, UART_IER_DLH, im);

}

#ifdef CONFIG_SG2002_UART_WAIT_LCR
/****************************************************************************
 * Name: up_wait
 *
 * Description:
 *   Wait until UART is not busy. This is needed before writing to LCR.
 *   Otherwise we will get spurious interrupts on Synopsys DesignWare 8250.
 *
 * Input Parameters:
 *   priv: UART Struct
 *
 * Returned Value:
 *   Zero (OK) on success; ERROR if timeout.
 *
 ****************************************************************************/

static int up_wait(FAR struct u16550_s *priv)
{
  int i;

  for (i = 0; i < UART_TIMEOUT_MS; i++)
    {
      uint32_t status = up_serialin(priv, UART_USR);

      if ((status & USR_UART_BUSY) == 0)
        {
          return OK;
        }

      up_mdelay(1);
    }

  _err("UART timeout\n");
  return ERROR;
}
#endif /* CONFIG_SG2002_UART_WAIT_LCR */

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static void up_disableuartint(struct up_dev_s *priv, uint8_t *im)
{
  /* Return the current interrupt mask value */

  if (im)
    {
     *im = priv->im;
    }

  /* Disable all interrupts */

  priv->im = 0;

  up_serialout(priv, UART_IER_DLH, 0);
}

/****************************************************************************
 * Name: up_enablebreaks
 ****************************************************************************/

static inline void up_enablebreaks(FAR struct up_dev_s *priv,
                                       bool enable)
{
  uint32_t lcr = up_serialin(priv, UART_LCR);

  if (enable)
    {
      lcr |= LCR_BREAK_CTRL_BIT;
    }
  else
    {
      lcr &= ~LCR_BREAK_CTRL_BIT;
    }

#ifdef CONFIG_SG2002_UART_WAIT_LCR
  /* Wait till UART is not busy before setting LCR */

  if (up_wait(priv) < 0)
    {
      _err("UART wait failed\n");
    }
#endif /* CONFIG_SG2002_UART_WAIT_LCR */

  up_serialout(priv, UART_LCR, lcr);
}

/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
  uint16_t div;
  double tmp;
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Software Reset */
  up_serialout(priv, UART_SRR, 0x7);

  /* Enable and configure the selected console device */
  up_serialout(priv, UART_IER_DLH, 0);
  up_serialout(priv, UART_MCR, 0);
  up_serialout(priv, UART_FCR_IIR, UART_FCR_DEFVAL);
  up_serialout(priv, UART_LCR, UART_LCR_BKSE | UART_LCRVAL);

  /*
   * Configure the UART Baud Rate setting Div
   *                 UART_SCLK
   *  Baud rate = _____________________
   *              16 ∗ (256 ∗ DLH + DLL)
   *
   * eg: baudrate: 115200
   *  256 * DLH + DLL = UART_SCLK/ (16 * 115200)
   */

  tmp = (CONFIG_UART_SCLK * 1.0) / (16 * priv->baud);
  div = ceil(tmp);

  up_serialout(priv, UART_RBR_THR_DLL, div & 0xFF);
  up_serialout(priv, UART_IER_DLH, div >> 8);

  /* 8n1 no parity */
  up_serialout(priv, UART_LCR, UART_LCRVAL);
  /* Enable RX */

  up_serialout(priv, UART_IER_DLH,
               IER_DLH_RECV_DATA | IER_DLH_RECV_LINE_STA);

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set up the interrupt priority */

  up_prioritize_irq(priv->irq, priv->irqprio);
#endif

  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disableuartint(priv, NULL);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode. This method is
 *   called when the serial port is opened.  Normally, this is just after the
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled by the attach method (unless the
 *   hardware supports multiple levels of interrupt enabling).  The RX and TX
 *   interrupts are not enabled until the txint() and rxint() are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ret = OK;

  /* Initialize interrupt generation on the peripheral */

  ret = irq_attach(priv->irq, up_interrupt, dev);

  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called. The exception
 *   is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable interrupts */

  up_disable_irq(priv->irq);

  /* Detach from the interrupt */

  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;
  uint32_t           status;
  int                passes;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  for (passes = 0; passes < 64; passes++)
    {
      /* Retrieve interrupt pending status */

      status = up_serialin(priv, UART_FCR_IIR);

      switch (status & 0xF)
	{

	  case FCR_IIR_NO_INT_PENDING:
            {
              return OK;
            }
          case FCR_IIR_RECV_DATA:
            {
              /* Process incoming bytes */

              uart_recvchars(dev);

	      break;
            }

          case FCR_IIR_THR_EMPTY:
            {
              /* Process outgoing bytes */

              uart_xmitchars(dev);

	      break;
            }

	  default:
            break;
	}
    }

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  struct inode *inode    = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  struct up_dev_s *priv  = (struct up_dev_s *)dev->priv;
  int ret;

#ifdef CONFIG_SERIAL_UART_ARCH_IOCTL
  ret = uart_ioctl(filep, cmd, arg);

  if (ret != -ENOTTY)
    {
      return ret;
    }

#else
  ret = OK;
#endif

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
        struct up_dev_s *user = (struct up_dev_s *)arg;
        if (!user)
          {
            ret = -EINVAL;
          }
        else
          {
            memcpy(user, dev, sizeof(struct up_dev_s));
          }
      }
      break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
      {
        irqstate_t flags = enter_critical_section();
        up_enablebreaks(priv, true);
        leave_critical_section(flags);
      }
      break;

    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
      {
        irqstate_t flags;
        flags = enter_critical_section();
        up_enablebreaks(priv, false);
        leave_critical_section(flags);
      }
      break;

#if defined(CONFIG_SERIAL_TERMIOS) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
    case TCGETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        irqstate_t flags;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        flags = enter_critical_section();

        cfsetispeed(termiosp, priv->baud);
        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);
        termiosp->c_cflag |= (priv->stopbits2) ? CSTOPB : 0;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        termiosp->c_cflag |= priv->flow ? CRTSCTS : 0;
#endif

        switch (priv->bits)
          {
          case 5:
            termiosp->c_cflag |= CS5;
            break;

          case 6:
            termiosp->c_cflag |= CS6;
            break;

          case 7:
            termiosp->c_cflag |= CS7;
            break;

          case 8:
          default:
            termiosp->c_cflag |= CS8;
            break;
          }

        leave_critical_section(flags);
      }
      break;

    case TCSETS:
      {
        struct termios *termiosp = (struct termios *)arg;
        irqstate_t flags;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        flags = enter_critical_section();

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            priv->bits = 5;
            break;

          case CS6:
            priv->bits = 6;
            break;

          case CS7:
            priv->bits = 7;
            break;

          case CS8:
          default:
            priv->bits = 8;
            break;
          }

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        priv->baud      = cfgetispeed(termiosp);
        priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;
#if defined(CONFIG_SERIAL_IFLOWCONTROL) || defined(CONFIG_SERIAL_OFLOWCONTROL)
        priv->flow      = (termiosp->c_cflag & CRTSCTS) != 0;
#endif

        up_setup(dev);
        leave_critical_section(flags);

#ifdef CONFIG_CLK
        /* Clk enable */

        priv->mclk = clk_get(priv->clk_name);
        if (priv->mclk)
          {
            clk_set_rate(priv->mclk, priv->uartclk);
          }
#endif
      }
      break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, uint32_t *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t rbr;

  *status = up_serialin(priv, UART_LSR);
  rbr     = up_serialin(priv, UART_RBR_THR_DLL);
  return rbr;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= IER_DLH_RECV_DATA | IER_DLH_RECV_LINE_STA;
#endif
    }
  else
    {
      priv->im &= ~(IER_DLH_RECV_DATA | IER_DLH_RECV_LINE_STA);
    }

  up_serialout(priv, UART_IER_DLH, priv->im);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t dr = 0;

  /* Return true is data is available in the receive data buffer */

  dr = up_serialin(priv, UART_LSR) & LSR_DATA_READY;

  return !!dr;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART.
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Wait until the TX data register is empty */

  while (!(up_serialin(priv, UART_LSR) & LSR_TRANS_HOLDING_REG_EMPTY))
    {
    }

  /* Then send the character */

  up_serialout(priv, UART_RBR_THR_DLL, ch);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
      /* Enable the TX interrupt */

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->im |= IER_DLH_TRANS_HOLDING_REG_EMPTY;
      up_serialout(priv, UART_IER_DLH, priv->im);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */

//      uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */

      priv->im &= ~IER_DLH_TRANS_HOLDING_REG_EMPTY;
      up_serialout(priv, UART_IER_DLH, priv->im);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit data register is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the TX FIFO is not full */

  return !!(up_serialin(priv, UART_LSR) & LSR_TRANS_HOLDING_REG_EMPTY);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the tranmsit data register is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Return TRUE if the TX wartermak is pending */

  return !!(up_serialin(priv, UART_LSR) & LSR_TRANS_HOLDING_REG_EMPTY);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: riscv_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before riscv_serialinit.  NOTE:  This function depends on GPIO pin
 *   configuration performed in up_consoleinit() and main clock iniialization
 *   performed in up_clkinitialize().
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
  /* Configuration whichever one is the console */

#ifdef HAVE_SERIAL_CONSOLE
  CONSOLE_DEV.isconsole = true;
  up_setup(&CONSOLE_DEV);
#endif
}
#endif

/****************************************************************************
 * Name: riscv_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that riscv_earlyserialinit was called previously.
 *
 ****************************************************************************/

void riscv_serialinit(void)
{
  /* Register the console */

#ifdef HAVE_SERIAL_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif

  /* Register all UARTs */

  uart_register("/dev/ttyS0", &TTYS0_DEV);
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
int up_putc(int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)CONSOLE_DEV.priv;

  irqstate_t flags;

  /* All interrupts must be disabled to prevent re-entrancy and to prevent
   * interrupts from firing in the serial driver code.
   */

  flags = enter_critical_section();
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      sg2002_putc(priv, '\r');
    }

  sg2002_putc(priv, ch);
  leave_critical_section(flags);

  return ch;
}
#endif

#else /* HAVE_UART_DEVICE */

/****************************************************************************
 * Name: riscv_earlyserialinit, riscv_serialinit, and up_putc
 *
 * Description:
 *   stubs that may be needed.  These stubs would be used if all UARTs are
 *   disabled.  In that case, the logic in common/up_initialize() is not
 *   smart enough to know that there are not UARTs and will still expect
 *   these interfaces to be provided.
 *
 ****************************************************************************/

void riscv_earlyserialinit(void)
{
}

void riscv_serialinit(void)
{
}

int up_putc(int ch)
{
  return ch;
}

#endif /* HAVE_UART_DEVICE */
#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
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

      riscv_lowputc('\r');
    }

  riscv_lowputc(ch);
#endif
  return ch;
}

#endif /* USE_SERIALDRIVER */
