/**
  ******************************************************************************
  * @file    task6_fsm.c
  * @brief   TASK 6 : NON-BLOCKING EEPROM TRANSACTION STATE MACHINE
  *
  * Restructure the Task 4 transaction so the main loop never stops:
  *
  *   request -> write-enable -> write -> wait for EEPROM
  *           -> read-back -> verify -> result
  *
  * Rules (handout, Task 6):
  *   - No HAL_Delay(), and no software busy-wait for the EEPROM's internal
  *     write cycle. You may use HAL_GetTick() to decide when the next status
  *     check is due.
  *   - Each call does a small amount of work, updates the state, and returns.
  *   - Do not put the whole transaction inside one blocking function called
  *     from the loop.
  *   - While a transaction is in progress the loop must still respond to PA3.
  *
  * Controls: PA0 starts, PA3 aborts. PB0..PB7 show the last byte read, PB11
  * (green) a successful verification, PB10 (red) a failed one.
  ******************************************************************************
  */

#include "prac2a.h"

/* TODO 6.1  Design your states. You need enough of them to tell apart at
 *           least: idle, write preparation, write transaction, EEPROM busy /
 *           status checking, read transaction, verification, and success or
 *           failure.
 *
 *           Draw the diagram first. It must show the initial state, the
 *           condition on every transition, and the success, failure and
 *           abort paths.
 *
 *           typedef enum { ... } ee_state_t;
 */

/* ===================== TODO 6.1 - ANSWER ================================ */

/* The number is what ee_state shows in Live Expressions.
 *
 *   0 IDLE       waiting for PA0                                (idle)
 *   1 PRECHECK   RDSR: EEPROM must not still be busy            (write prep)
 *   2 WREN       send write-enable                              (write prep)
 *   3 CHECK_WEL  RDSR: confirm WEL bit = 1                      (write prep)
 *   4 WRITE      WRITE + address + data, CS high, DON'T wait    (write transaction)
 *   5 WAIT_BUSY  RDSR once every 1 ms until RDY bit = 0         (busy / status check)
 *   6 READ       READ + address + dummy -> byte                 (read transaction)
 *   7 VERIFY     read-back == written ?                         (verification)
 *
 * After VERIFY (or any failure / abort) the machine goes back to IDLE and
 * the outcome is kept in ee_result -> green / red LED.   (success / failure)
 */
typedef enum
{
    EE_IDLE      = 0,
    EE_PRECHECK  = 1,
    EE_WREN      = 2,
    EE_CHECK_WEL = 3,
    EE_WRITE     = 4,
    EE_WAIT_BUSY = 5,
    EE_READ      = 6,
    EE_VERIFY    = 7
} ee_state_t;

typedef enum
{
    EE_RES_NONE    = 0,     /* nothing yet                   */
    EE_RES_BUSY    = 1,     /* transaction in progress       */
    EE_RES_OK      = 2,     /* verified        -> green      */
    EE_RES_FAIL    = 3,     /* failed          -> red        */
    EE_RES_ABORTED = 4      /* PA3 pressed     -> both off   */
} ee_result_t;

typedef enum                /* why it failed - debugger / report only */
{
    EE_ERR_NONE          = 0,
    EE_ERR_PRE_TIMEOUT   = 1,   /* EEPROM already busy for too long  */
    EE_ERR_WEL_NOT_SET   = 2,   /* WREN not accepted                 */
    EE_ERR_WRITE_TIMEOUT = 3,   /* never reported ready after write  */
    EE_ERR_MISMATCH      = 4    /* read-back != byte written         */
} ee_err_t;

/* Values Task 4 did not need, so board_config.h may not have them.
 * Given local names so they cannot clash with anything in board_config.h. */
#define EE_CMD_WRDI     0x04u   /* write-disable instruction            */
#define EE_SR_WEL       0x02u   /* status bit 1: write-enable latch      */
#define EE_POLL_MS      1u      /* time between status checks when busy  */

/* ======================================================================== */

volatile uint8_t ee_state       = 0u;
volatile uint8_t ee_last_read   = 0u;
volatile uint8_t ee_use_fsm     = 1u;

/* Added for Live Expressions */
volatile uint8_t  ee_result      = EE_RES_NONE;
volatile uint8_t  ee_err         = EE_ERR_NONE;
volatile uint32_t ee_polls       = 0u;  /* status checks during last write */

/* DEMO AID, writable in Live Expressions. The whole transaction takes ~6 ms,
 * too fast to watch ee_state or to press PA3 in the middle. Set to 1000 and
 * each state is held for 1 s - still non-blocking (a tick comparison, then
 * return). Set back to 0 for normal speed. */
volatile uint32_t ee_demo_step_ms = 0u;

/* State machine memory between calls */
static uint32_t t_start;    /* when the current wait began           */
static uint32_t t_next;     /* when the next status check is due     */
static uint32_t t_hold;     /* demo aid: next step not before this   */
static uint16_t tx_addr;    /* copied at start so a Live-Expression  */
static uint8_t  tx_data;    /* edit mid-transaction can't break it   */


/* ===================== helpers for TODO 6.2 ==============================
 *
 * REUSED FROM TASK 4 unchanged (each is already one short transaction):
 *   eeprom_read_status()   2 bytes   ~64 us
 *   eeprom_write_enable()  1 byte    ~32 us
 *   eeprom_read_byte()     4 bytes  ~128 us
 *
 * NOT reused: eeprom_write_byte(). It contains the Task 4 while-loop that
 * waits ~5 ms for the write cycle - exactly what Task 6 forbids. So below is
 * its first half only (the SPI frame), and the waiting half becomes the
 * WAIT_BUSY state.
 * ======================================================================== */

/* First half of Task 4's eeprom_write_byte(): send the frame, don't wait */
static void ee_send_write_frame(uint16_t address, uint8_t value)
{
    eeprom_cs_low();
    (void)spi_transfer(EEPROM_CMD_WRITE);
    for (uint32_t i = EEPROM_ADDR_BYTES; i > 0u; i--)       /* MSB first */
    {
        (void)spi_transfer((uint8_t)(address >> (8u * (i - 1u))));
    }
    (void)spi_transfer(value);
    eeprom_cs_high();           /* internal write starts here - we return */
}

/* Write-disable: one byte, own transaction (used on abort) */
static void ee_write_disable(void)
{
    eeprom_cs_low();
    (void)spi_transfer(EE_CMD_WRDI);
    eeprom_cs_high();
}

/* Status LEDs off (in progress / aborted). Resets PB10 and PB11 through
 * BSRR, same method as leds_write_byte(). If status_leds_show() in
 * board_io.c has an "off" value (check the STATUS_ enum in prac2a.h), you
 * can use that instead. */
static void ee_status_leds_off(void)
{
    GPIOB->BSRR = (1u << (10u + 16u)) | (1u << (11u + 16u));
}

/* Wrap-safe "has tick t arrived?" */
static uint8_t time_reached(uint32_t now, uint32_t t)
{
    return (int32_t)(now - t) >= 0;
}

/* End a transaction: store result, back to IDLE */
static void finish(ee_result_t result, ee_err_t err)
{
    ee_result = result;
    ee_err    = err;
    ee_state  = EE_IDLE;
}


void update_eeprom_state_machine(uint32_t now)
{
    /* TODO 6.2  One step of your state machine.
     *
     *   - btn_start_edge (PA0) starts a transaction from idle.
     *   - btn_abort_edge (PA3) abandons the transaction in progress and returns
     *     to idle. Leave the SPI bus in a state the next transaction can use.
     *   - While the EEPROM is busy writing, do NOT wait in here. Work out when
     *     the next status check is due, remember it, and return.
     *   - Decide that the write has finished from the status register, never
     *     from elapsed time alone. Also decide what happens if the EEPROM never
     *     reports ready.
     *   - Store the byte read back in ee_last_read.
     *   - Clear each button edge once you have acted on it. */

    /* ===================== TODO 6.2 - ANSWER ============================ */

    uint8_t prev_state;

    /* ---- A. ABORT first, so PA3 works in every state ------------------- */
    if (btn_abort_edge)
    {
        btn_abort_edge = 0u;

        if (ee_state != EE_IDLE)
        {
            /* Every state finishes its SPI transaction before returning,
             * so CS is already high and the bus is idle. WRDI clears WEL in
             * case we stopped between WREN and WRITE. If WRITE was already
             * sent, the EEPROM completes that write internally anyway (an
             * abort can't undo it) - PRECHECK handles that next time. */
            ee_write_disable();
            finish(EE_RES_ABORTED, EE_ERR_NONE);
        }
        return;
    }

    /* ---- B. Demo slow-down (inactive when ee_demo_step_ms = 0) --------- */
    if ((ee_state != EE_IDLE) && !time_reached(now, t_hold))
    {
        btn_start_edge = 0u;
        return;
    }

    prev_state = ee_state;

    /* ---- C. One state's worth of work, then return --------------------- */
    switch ((ee_state_t)ee_state)
    {
    case EE_IDLE:
        if (btn_start_edge)
        {
            btn_start_edge = 0u;
            tx_addr   = eeprom_test_addr;
            tx_data   = eeprom_test_byte;
            if (tx_addr >= EEPROM_SIZE_BYTES)       /* same guard as Task 4 */
            {
                finish(EE_RES_FAIL, EE_ERR_NONE);
                break;
            }
            ee_result = EE_RES_BUSY;
            ee_err    = EE_ERR_NONE;
            t_start   = now;
            t_next    = now;                        /* check immediately */
            ee_state  = EE_PRECHECK;
        }
        break;

    case EE_PRECHECK:
        /* The EEPROM ignores WREN while a write cycle is running (e.g. one
         * we aborted), so make sure it's ready first. */
        if (!time_reached(now, t_next))
        {
            break;
        }
        eeprom_status_before = eeprom_read_status();
        if ((eeprom_status_before & EEPROM_SR_RDY) == 0u)
        {
            ee_state = EE_WREN;
        }
        else if ((uint32_t)(now - t_start) >= EEPROM_WRITE_TIMEOUT_MS)
        {
            finish(EE_RES_FAIL, EE_ERR_PRE_TIMEOUT);
        }
        else
        {
            t_next = now + EE_POLL_MS;
        }
        break;

    case EE_WREN:
        eeprom_write_enable();                  /* Task 4 function */
        ee_state = EE_CHECK_WEL;
        break;

    case EE_CHECK_WEL:
        if (eeprom_read_status() & EE_SR_WEL)
        {
            ee_state = EE_WRITE;
        }
        else
        {
            finish(EE_RES_FAIL, EE_ERR_WEL_NOT_SET);
        }
        break;

    case EE_WRITE:
        ee_send_write_frame(tx_addr, tx_data);
        ee_polls = 0u;
        t_start  = now;
        t_next   = now + EE_POLL_MS;
        ee_state = EE_WAIT_BUSY;
        break;

    case EE_WAIT_BUSY:
        /* This replaces Task 4's  while (status & RDY) { ... }  loop.
         * One check per visit at most, and only when it is due. */
        if (!time_reached(now, t_next))
        {
            break;                              /* not due: return */
        }
        ee_polls++;
        eeprom_status_after = eeprom_read_status();
        if ((eeprom_status_after & EEPROM_SR_RDY) == 0u)
        {
            eeprom_write_wait_ms = (uint32_t)(now - t_start);
            ee_state = EE_READ;                 /* decided by status reg */
        }
        else if ((uint32_t)(now - t_start) >= EEPROM_WRITE_TIMEOUT_MS)
        {
            eeprom_timeout_count++;             /* same counter as Task 4 */
            finish(EE_RES_FAIL, EE_ERR_WRITE_TIMEOUT);
        }
        else
        {
            t_next = now + EE_POLL_MS;          /* still busy: 1 ms later */
        }
        break;

    case EE_READ:
        ee_last_read      = eeprom_read_byte(tx_addr);  /* Task 4 function */
        eeprom_read_value = ee_last_read;
        ee_state          = EE_VERIFY;
        break;

    case EE_VERIFY:
        eeprom_verify_ok = (ee_last_read == tx_data) ? 1u : 0u;
        if (eeprom_verify_ok)
        {
            finish(EE_RES_OK, EE_ERR_NONE);
        }
        else
        {
            finish(EE_RES_FAIL, EE_ERR_MISMATCH);
        }
        break;

    default:                                    /* corrupted state value */
        eeprom_cs_high();
        finish(EE_RES_FAIL, EE_ERR_NONE);
        break;
    }

    /* PA0 during a transaction is ignored; clear it so it can't fire later */
    if (ee_state != EE_IDLE)
    {
        btn_start_edge = 0u;
    }

    /* Demo aid: hold each new state for ee_demo_step_ms */
    if (ee_state != prev_state)
    {
        t_hold = now + ee_demo_step_ms;
    }

    /* ==================================================================== */
}

void update_outputs(void)
{
    /* TODO 6.3  PB0..PB7 show ee_last_read (leds_write_byte). Green after a
     *           successful verification, red after a failed one
     *           (status_leds_show). Decide what the status LEDs should show
     *           while a transaction is in progress, and after an abort. */

    /* ===================== TODO 6.3 - ANSWER ============================ */

    leds_write_byte(ee_last_read);

    switch ((ee_result_t)ee_result)
    {
    case EE_RES_OK:   status_leds_show(STATUS_PASS); break;   /* green     */
    case EE_RES_FAIL: status_leds_show(STATUS_FAIL); break;   /* red       */
    default:          ee_status_leds_off();          break;   /* busy/abort */
    }

    /* ==================================================================== */
}

/* ==========================================================================
 * RUN_TASK 6 - given
 *
 * The main loop has exactly the shape the handout asks for:
 *     read_inputs();  update_eeprom_state_machine();  update_outputs();
 *
 * Set ee_use_fsm = 0 in Live Expressions to run the Task 4 blocking path on
 * PA0 instead - useful when you explain why yours is non-blocking. PC13 keeps
 * toggling as a heartbeat; watch it on the scope in both modes.
 * ========================================================================== */

void task6_setup(void)
{
    task1_gpio_init();
    eeprom_spi_init();
    board_io_init();

    eeprom_read_only_path();
    ee_last_read = eeprom_read_value;

    /* TODO 6.4  Your state machine starts in its idle state. Make sure the
     *           boot-time result (eeprom_verify_ok) still shows on the status
     *           LEDs, so a reset still shows green for the persistence test. */

    /* ===================== TODO 6.4 - ANSWER ============================ */

    ee_state  = EE_IDLE;
    ee_result = eeprom_verify_ok ? EE_RES_OK : EE_RES_FAIL;
    ee_err    = EE_ERR_NONE;
    btn_start_edge = 0u;
    btn_abort_edge = 0u;

    /* ==================================================================== */
}

void task6_loop(uint32_t now)
{
    task1_gpio_update(now);
    read_inputs(now);

    if (ee_use_fsm)
    {
        update_eeprom_state_machine(now);
        update_outputs();
    }
    else
    {
        if (btn_start_edge)
        {
            btn_start_edge = 0u;
            eeprom_write_verify_path();     /* Task 4: blocks for the write */
            ee_last_read = eeprom_read_value;
        }
        btn_abort_edge = 0u;
    }
}
