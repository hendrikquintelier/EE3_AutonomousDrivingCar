/* -------------------------------------------------------------------------
 * barcode.c – IR barcode helper implementation
 * -------------------------------------------------------------------------
 * This module watches the IR‑phototransistor voltage while the vehicle
 * drives over a floor‑mounted 8‑bit barcode sticker.
 *
 *  1.  During the first GREY_LENGTH_CM centimetres we learn the reference
 *      grey level (μ, σ).
 *  2.  Once |v‑μ| > DEVIATION_MULT·σ for DEVIATION_COUNT consecutive
 *      samples we conclude that the grey band is left and start recording
 *      (state → ST_RECORD).  The ring‑buffer that held the outliers is
 *      copied into the main waveform buffer so that *all* black/white
 *      transitions are kept.
 *  3.  While recording, each sample is checked against the grey threshold
 *      again.  Once BACK_COUNT consecutive in‑grey samples are seen we
 *      assume that the grey band is reached again → ST_DONE.
 *  4.  In ST_DONE the recorded distance range is sliced into 8 equal
 *      windows and converted to ASCII bits in B.bits[] (bc_finish()).
 *
 *  NOTE:  The caller must pass *volts* (not raw ADC counts) to
 *         barcode_update().  Conversion is done in adc.c right after the
 *         reading is taken.
 * -------------------------------------------------------------------------
 */

#include "barcode.h"
#include "wifi_logger.h"
#include <math.h>
#include <string.h>

/* --------------------------- Tunables ---------------------------------- */
#define GREY_LENGTH_CM 10.0f  /* learn μ,σ over first 10 cm          */
#define DEVIATION_COUNT 3     /* need 3 consecutive outliers to start*/
#define BACK_COUNT 5          /* need 2 consecutive grey samples to stop */
#define DEVIATION_MULT 6.0f   /* |v‑μ| > 3σ                          */
#define MAX_CODE_SAMPLES 2000 /* plenty for 50 Hz * 40 cm @ 10 cm/s  */

/* --------------------------- State machine ----------------------------- */
typedef enum
{
    ST_GREY = 0,
    ST_WAIT_DEV,
    ST_RECORD,
    ST_DONE
} bc_state_t;

static struct
{
    /* --------------- grey statistics --------------- */
    float grey_sum;
    float grey_sum2;
    int grey_n;
    float mu;
    float sigma;

    /* --------------- deviation detector ------------ */
    float ring_v[DEVIATION_COUNT];
    float ring_d[DEVIATION_COUNT];
    int ring_idx;
    int dev_cnt;

    /* --------------- back‑to‑grey detector --------- */
    int back_cnt;

    /* --------------- recorded waveform ------------- */
    float code_v[MAX_CODE_SAMPLES];
    float code_d[MAX_CODE_SAMPLES];
    int code_n;

    /* --------------- result ------------------------ */
    char bits[9];

    bc_state_t st;
} B;

/* --------------------------------------------------------------------- */
void barcode_reset(void)
{
    memset(&B, 0, sizeof(B));
    B.st = ST_GREY;
    log_remote("[BARCODE] reset → ST_GREY");
}

/* --------------------------------------------------------------------- */
void barcode_update(float d_cm, float v)
{
    switch (B.st)
    {
    /* ------------------------- 1. learn μ,σ --------------------------- */
    case ST_GREY:
        if (d_cm <= GREY_LENGTH_CM)
        {
            B.grey_sum += v;
            B.grey_sum2 += v * v;
            B.grey_n++;
            if ((int)B.grey_n % 5 == 0) /* throttle logging */
                log_remote("[BARCODE] sample d≈%.0f cm  v=%.3f V  st=%d", d_cm, v, B.st);
        }
        else if (B.grey_n >= 5) /* avoid div/0 */
        {
            B.mu = B.grey_sum / (float)B.grey_n;
            float var = (B.grey_sum2 / (float)B.grey_n) - B.mu * B.mu;
            B.sigma = sqrtf(fmaxf(var, 1e-6f));
            log_remote("[BARCODE] μ=%.3f σ=%.3f (grey_n=%d)", B.mu, B.sigma, B.grey_n);
            log_remote("[BARCODE] ST_GREY → ST_WAIT_DEV");
            B.st = ST_WAIT_DEV;
        }
        break;

    /* --------------------- 2. wait for deviation ---------------------- */
    case ST_WAIT_DEV:
    {
        /* push into ring buffer */
        B.ring_v[B.ring_idx] = v;
        B.ring_d[B.ring_idx] = d_cm;
        B.ring_idx = (B.ring_idx + 1) % DEVIATION_COUNT;

        const float thr = DEVIATION_MULT * B.sigma;
        const float dev = fabsf(v - B.mu);
        log_remote("[BARCODE] WAIT_DEV  |v-μ|=%.3f thr=%.3f  dev=%d", dev, thr, B.dev_cnt);
        if (dev > thr)
        {
            if (++B.dev_cnt >= DEVIATION_COUNT)
            {
                /* copy oldest→newest into main buffer */
                int start = B.ring_idx;
                for (int i = 0; i < DEVIATION_COUNT; ++i)
                {
                    int idx = (start + i) % DEVIATION_COUNT;
                    if (B.code_n < MAX_CODE_SAMPLES)
                    {
                        B.code_v[B.code_n] = B.ring_v[idx];
                        B.code_d[B.code_n] = B.ring_d[idx];
                        B.code_n++;
                    }
                }
                log_remote("[BARCODE] deviation confirmed (cnt=%d) → ST_RECORD", B.dev_cnt);
                B.st = ST_RECORD;
                B.back_cnt = 0;
            }
        }
        else
        {
            B.dev_cnt = 0; /* reset window */
        }
        break;
    }

    /* --------------------- 3. record waveform ------------------------- */
    case ST_RECORD:
    {
        /* classify current sample */
        const float thr = DEVIATION_MULT * B.sigma;
        int is_grey = (fabsf(v - B.mu) <= thr);
        if (is_grey)
        {
            B.back_cnt++;
        }
        else
        {
            B.back_cnt = 0;
            /* only record non-grey samples */
            if (B.code_n < MAX_CODE_SAMPLES)
            {
                B.code_v[B.code_n] = v;
                B.code_d[B.code_n] = d_cm;
                B.code_n++;
            }
        }

        log_remote("[BARCODE] RECORD d=%.2f v=%.3f grey=%d back=%d", d_cm, v, is_grey, B.back_cnt);

        if (B.back_cnt >= BACK_COUNT)
        {
            log_remote("[BARCODE] grey regained (%d×) → DONE", BACK_COUNT);
            B.st = ST_DONE;
            bc_finish();
        }
        break;
    }

    /* --------------------- 4. done ------------------------------------ */
    case ST_DONE:
    default:
        /* nothing to do */
        break;
    }
}

/* --------------------------------------------------------------------- */
bool barcode_is_complete(void)
{
    return (B.st == ST_DONE);
}

const char *barcode_get_bits(void)
{
    log_remote("[BARCODE] bits      : %s", B.bits);
    log_remote("[BARCODE] μ,σ       : %.3f V  %.3f V", B.mu, B.sigma);
    log_remote("[BARCODE] samples   : %d", B.code_n);
    return B.bits;
}

/* --------------------------------------------------------------------- */
void bc_finish(void)
{
    if (B.code_n < 2)
        return; /* not enough data */

    const float start_d = B.code_d[0];
    const float end_d = B.code_d[B.code_n - 1];
    const float span = end_d - start_d;
    const float win = span / 8.0f;

    log_remote("[BARCODE] span=%.2f cm  win=%.2f cm", span, win);

    for (int b = 0; b < 8; ++b)
    {
        const float d0 = start_d + b * win;
        const float d1 = d0 + win;
        int cnt0 = 0, cnt1 = 0;
        for (int i = 0; i < B.code_n; ++i)
        {
            if (B.code_d[i] >= d0 && B.code_d[i] < d1)
            {
                if (B.code_v[i] > B.mu)
                    cnt1++;
                else
                    cnt0++;
            }
        }
        B.bits[b] = (cnt1 >= cnt0 ? '1' : '0');
        log_remote("[BARCODE] bit %d  range %.2f–%.2f cm  1:%d 0:%d  -> %c", b, d0, d1, cnt1, cnt0, B.bits[b]);
    }
    B.bits[8] = '\0';
    log_remote("[BARCODE] FINAL = %s", B.bits);
}
