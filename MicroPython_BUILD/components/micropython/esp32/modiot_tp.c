#include <stdio.h>
#include "py/gc.h"
#include "py/runtime.h"
#include "esp_log.h"
#include "iot_touchpad.h"

#define S0 TOUCH_PAD_NUM0
#define S1 TOUCH_PAD_NUM1
#define S2 TOUCH_PAD_NUM2
#define S3 TOUCH_PAD_NUM3
#define S4 TOUCH_PAD_NUM4
#define S5 TOUCH_PAD_NUM5
#define S6 TOUCH_PAD_NUM6
#define S7 TOUCH_PAD_NUM7
#define S8 TOUCH_PAD_NUM8

#define LEFT   TOUCH_PAD_NUM0
// S1 defined above keeps same name
#define UP     TOUCH_PAD_NUM2 
#define DOWN   TOUCH_PAD_NUM3 
#define RIGHT  TOUCH_PAD_NUM4 
#define SELECT TOUCH_PAD_NUM5
#define START  TOUCH_PAD_NUM6
#define A      TOUCH_PAD_NUM7
#define B      TOUCH_PAD_NUM8

static const char *TAG = "iot_touchpad";

// Decrease for greater sensitivity
// compute with: (untouched_read - touched_read)/untouched_read
/*
#define S0_SENS  0.61  // ( 611  - 116) / 611  = 0.8101
#define S1_SENS  0.97  // ( 1070 - 1)   / 1070 = 0.9991
#define S2_SENS  0.82  // ( 598  - 110) / 598  = 0.8161
#define S3_SENS  0.71  // ( 508  - 146) / 508  = 0.7126
#define S4_SENS  0.64  // ( 512  - 182) / 512  = 0.6445
#define S5_SENS  0.68  // ( 1109 - 350) / 1109 = 0.6844
#define S6_SENS  0.73  // ( 939  - 250) / 939  = 0.7337
#define S7_SENS  0.33  // ( 540  - 200) / 540  = 0.6296
#define S8_SENS  0.36  // ( 569  - 250) / 569  = 0.5606
*/

#define SENS_MOD 0.0  // increase sensitivity by setting higher(0->0.999)
#define S0_SENS  0.2  // ( 611  - 116) / 611  = 0.8101
#define S1_SENS  0.2  // ( 1070 - 1)   / 1070 = 0.9991
#define S2_SENS  0.2  // ( 598  - 110) / 598  = 0.8161
#define S3_SENS  0.2  // ( 508  - 146) / 508  = 0.7126
#define S4_SENS  0.2  // ( 512  - 182) / 512  = 0.6445
#define S5_SENS  0.2  // ( 1109 - 350) / 1109 = 0.6844
#define S6_SENS  0.2  // ( 939  - 250) / 939  = 0.7337
#define S7_SENS  0.2  // ( 540  - 200) / 540  = 0.6296
#define S8_SENS  0.2  // ( 569  - 250) / 569  = 0.5606

#define TP_NUM_PADS 9

static int   tp_nums[TP_NUM_PADS] = { S0, S1, S2, S3, S4, S5, S6, S7, S8 };
static float tp_sens[TP_NUM_PADS] = { S0_SENS, S1_SENS, S2_SENS, S3_SENS,
                                      S4_SENS, S5_SENS, S6_SENS, S7_SENS,
                                      S8_SENS };
static tp_handle_t tp_devs[TP_NUM_PADS];

typedef enum tp_evt_t {
  TP_PUSH,        // INITIAL PRESS
  TP_RELEASE,     // RELEASE
  TP_TAP,         // PUSH + RELEASE
  TP_PUSH_1S,     // LONG PRESS (1s) 
  TP_PUSH_3S,     //  ""  ""    (3s) 
  TP_PUSH_5S,     //  ""  ""    (5s)
  TP_PUSH_10S     //  ""  ""    (10s)
} tp_evt_t;

// In case callback is a bound method
typedef struct _mp_obj_bound_meth_t {
    mp_obj_base_t base;
    mp_obj_t meth;
    mp_obj_t self;
} mp_obj_bound_meth_t;

//-----------------------------------------------------------------------------

static mp_obj_t event_callback = NULL;

//-----------------------------------------------------------------------------
static void tp_cb_handler(void *arg, tp_evt_t evt_type)
{
    tp_handle_t tp_dev = (tp_handle_t) arg;
    touch_pad_t tp_num = iot_tp_num_get(tp_dev);
    //ESP_LOGI(TAG, "cb->tp=%d type=%d", tp_num, evt_type);
    mp_obj_bound_meth_t *o = NULL;
    mp_sched_carg_t *carg = NULL;

    if (event_callback) {
        carg = make_cargs(MP_SCHED_CTYPE_TUPLE);

        if (carg == NULL) return;

        if (!make_carg_entry(carg, 0, MP_SCHED_ENTRY_TYPE_INT, 
                             tp_num, NULL, NULL)) return;

        if (!make_carg_entry(carg, 1, MP_SCHED_ENTRY_TYPE_INT, 
                             evt_type, NULL, NULL)) return;
    }

    if (carg) {
        if (MP_OBJ_IS_FUN(event_callback)) {
            mp_sched_schedule(event_callback, mp_const_none, carg);
        }
        if (MP_OBJ_IS_METH(event_callback)) {
            o = MP_OBJ_TO_PTR(event_callback);
            mp_sched_schedule(event_callback, o->self , carg);
        }
    }
}

static void push_cb    (void *arg) { tp_cb_handler(arg, TP_PUSH)    ; }
static void release_cb (void *arg) { tp_cb_handler(arg, TP_RELEASE) ; }
static void tap_cb     (void *arg) { tp_cb_handler(arg, TP_TAP)     ; }
static void push_1s_cb (void *arg) { tp_cb_handler(arg, TP_PUSH_1S) ; }
static void push_3s_cb (void *arg) { tp_cb_handler(arg, TP_PUSH_3S) ; }
static void push_5s_cb (void *arg) { tp_cb_handler(arg, TP_PUSH_5S) ; }
static void push_10s_cb(void *arg) { tp_cb_handler(arg, TP_PUSH_10S); }

//-----------------------------------------------------------------------------
// no args: true if callback set, false if not
// 1  arg: set callback to arg1
//-----------------------------------------------------------------------------
STATIC mp_obj_t iot_tp_callback(size_t n_args, const mp_obj_t *args)
{
    if (n_args == 0) {
      if (event_callback == NULL) return mp_const_false;
      return mp_const_true;
    }

    if ((MP_OBJ_IS_FUN(args[0])) || (MP_OBJ_IS_METH(args[0]))) {
      event_callback = args[0];
    }
    else event_callback = NULL;

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(iot_tp_callback_obj,0,1,iot_tp_callback);

//-----------------------------------------------------------------------------
STATIC mp_obj_t iot_init_tp(void)
{
    int i;

    for ( i=0; i < TP_NUM_PADS; i++)
    {
      tp_devs[i] = iot_tp_create(tp_nums[i], tp_sens[i]-(tp_sens[i]*(SENS_MOD)));

      iot_tp_add_cb(tp_devs[i], TOUCHPAD_CB_PUSH, push_cb, tp_devs[i]);
      iot_tp_add_cb(tp_devs[i], TOUCHPAD_CB_RELEASE, release_cb, tp_devs[i]);
      iot_tp_add_cb(tp_devs[i], TOUCHPAD_CB_TAP, tap_cb, tp_devs[i]);
      iot_tp_add_custom_cb( tp_devs[i], 1, push_1s_cb, tp_devs[i]);
      iot_tp_add_custom_cb( tp_devs[i], 3, push_3s_cb, tp_devs[i]);
      iot_tp_add_custom_cb( tp_devs[i], 5, push_5s_cb, tp_devs[i]);
      iot_tp_add_custom_cb( tp_devs[i], 10, push_10s_cb, tp_devs[i]);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(iot_init_tp_obj, iot_init_tp);

//-----------------------------------------------------------------------------
STATIC mp_obj_t iot_deinit_tp(void)
{
    int i;

    event_callback = NULL;

    for (i=0; i < TP_NUM_PADS; i++) 
    {
        if (tp_devs[i] != NULL) {
        iot_tp_delete(tp_devs[i]);
        //ESP_LOGI(TAG,"touchpad %d deleted",i);
        tp_devs[i] = NULL ; 
        gc_collect(0);
        ESP_LOGI(TAG,"gc_collect");
        }
        //else ESP_LOGI(TAG,"touchpad %d was NULL.",i);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(iot_deinit_tp_obj,iot_deinit_tp);

//-----------------------------------------------------------------------------
// Get the current state of the touchpads
STATIC mp_obj_t iot_tp_state(void)
{
    unsigned int         i;
    tp_handle_t         tp;
    tp_status_t  tp_status;
    mp_obj_t     tuple[TP_NUM_PADS];
    esp_err_t          err;

    //ESP_LOGI(TAG, "iot_tp_get_state()");

    for ( i = 0; i < TP_NUM_PADS; i++ )
    {
        tp  = tp_devs[i];
        err = iot_tp_get_status(tp, &tp_status); 
        if (err != ESP_OK) {
            mp_raise_ValueError("Touchpad read error");
        }
        //tuple[i] = ( tp_status > 0 ) ? mp_const_true : mp_const_false ; 
        tuple[i] = mp_obj_new_int(tp_status);
        //ESP_LOGI(TAG, "touchpad %d state=%d", i, tp_status);
    }
    return mp_obj_new_tuple(TP_NUM_PADS, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(iot_tp_state_obj, iot_tp_state);

//-----------------------------------------------------------------------------
STATIC const mp_rom_map_elem_t iot_tp_globals_table[] = {
{ MP_ROM_QSTR(MP_QSTR___name__)  , MP_ROM_QSTR(MP_QSTR_iot_touchpad) },
{ MP_ROM_QSTR(MP_QSTR_init)      , MP_ROM_PTR(&iot_init_tp_obj)},
{ MP_ROM_QSTR(MP_QSTR_deinit)    , MP_ROM_PTR(&iot_deinit_tp_obj)},
{ MP_ROM_QSTR(MP_QSTR_callback)  , MP_ROM_PTR(&iot_tp_callback_obj)},
{ MP_ROM_QSTR(MP_QSTR_state)     , MP_ROM_PTR(&iot_tp_state_obj)},
{ MP_ROM_QSTR(MP_QSTR_LEFT)      , MP_ROM_INT(LEFT)   },
{ MP_ROM_QSTR(MP_QSTR_S1)        , MP_ROM_INT(S1)     },
{ MP_ROM_QSTR(MP_QSTR_UP)        , MP_ROM_INT(UP)     },
{ MP_ROM_QSTR(MP_QSTR_DOWN)      , MP_ROM_INT(DOWN)   },
{ MP_ROM_QSTR(MP_QSTR_RIGHT)     , MP_ROM_INT(RIGHT)  },
{ MP_ROM_QSTR(MP_QSTR_SELECT)    , MP_ROM_INT(SELECT) },
{ MP_ROM_QSTR(MP_QSTR_START)     , MP_ROM_INT(START)  },
{ MP_ROM_QSTR(MP_QSTR_A)         , MP_ROM_INT(A)      },
{ MP_ROM_QSTR(MP_QSTR_B)         , MP_ROM_INT(B)      },
{ MP_ROM_QSTR(MP_QSTR_PUSH)      , MP_ROM_INT(TP_PUSH)     },
{ MP_ROM_QSTR(MP_QSTR_RELEASE)   , MP_ROM_INT(TP_RELEASE ) },
{ MP_ROM_QSTR(MP_QSTR_TAP)       , MP_ROM_INT(TP_TAP)      },
{ MP_ROM_QSTR(MP_QSTR_PUSH_1S)   , MP_ROM_INT(TP_PUSH_1S)  },
{ MP_ROM_QSTR(MP_QSTR_PUSH_3S)   , MP_ROM_INT(TP_PUSH_3S)  },
{ MP_ROM_QSTR(MP_QSTR_PUSH_5S)   , MP_ROM_INT(TP_PUSH_5S)  },
{ MP_ROM_QSTR(MP_QSTR_PUSH_10S)  , MP_ROM_INT(TP_PUSH_10S) },
};
STATIC MP_DEFINE_CONST_DICT(iot_tp_module_globals, iot_tp_globals_table);

//-----------------------------------------------------------------------------
const mp_obj_module_t mp_module_iot_touchpad = {
    .base    = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&iot_tp_module_globals,
};
