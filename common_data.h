#ifndef common_datah
#define common_datah

#include <irsdefs.h>

#include <mxdata.h>

#include <irsfinal.h>

// Определения для сетевого массива

// Команды управления реле
// В младших 4-х битах номер катушки (нормального элемента), начиная с 1
// 0 - эталонная катушка (нормальный элемент)
// Включение прямого управления реле
#define RELAY_COM_DIRECT_CTRL    0x00
// Остановка измерений
#define RELAY_COM_STOP           0x10
// Измерение сопротивления
#define RELAY_COM_RES            0x20
// Измерение сопротивления с добавочным резистором 5 Ом
#define RELAY_COM_RES_RADD       0x30
// Прямое измерение напряжение на нормальном элементе
#define RELAY_COM_DIRECT_CELL    0x40
// Измерение напряжение на нормальном элементе методом сравнения
#define RELAY_COM_COMPARE_CELL   0x50
// Измерение сопротивления магазина
#define RELAY_COM_RES_BOX        0x60
// Прямое измерение напряжение на нормальном элементе
// с добавочным резистором 100 МОм
#define RELAY_COM_DIRECT_CELL_RADD 0x70

// Прямое управление реле
// Первые 4 бита - номер байта, вторые 4 бита - номер бита
// Включение реле S1I
#define RELAY_COM_RELAY_S1I       0x10
// Включение реле S2I
#define RELAY_COM_RELAY_S2I       0x11
// Включение реле S3I
#define RELAY_COM_RELAY_S3I       0x12
// Включение реле S4I
#define RELAY_COM_RELAY_S4I       0x13
// Включение реле S5I
#define RELAY_COM_RELAY_S5I       0x14
// Включение реле S6I
#define RELAY_COM_RELAY_S6I       0x15
// Включение реле S7I
#define RELAY_COM_RELAY_S7I       0x16
// Включение реле S8I
#define RELAY_COM_RELAY_S8I       0x17

// Включение реле S1U
#define RELAY_COM_RELAY_S1U       0x20
// Включение реле S2U
#define RELAY_COM_RELAY_S2U       0x21
// Включение реле S3U
#define RELAY_COM_RELAY_S3U       0x22
// Включение реле S4U
#define RELAY_COM_RELAY_S4U       0x23
// Включение реле S5U
#define RELAY_COM_RELAY_S5U       0x24
// Включение реле S6U
#define RELAY_COM_RELAY_S6U       0x25
// Включение реле S7U
#define RELAY_COM_RELAY_S7U       0x26
// Включение реле S8U
#define RELAY_COM_RELAY_S8U       0x27

// Включение реле SI
#define RELAY_COM_RELAY_SI        0x30
// Включение реле S0U
#define RELAY_COM_RELAY_S0U       0x31
// Включение реле SEU
#define RELAY_COM_RELAY_SEU       0x32
// Включение реле SU
#define RELAY_COM_RELAY_SU        0x33
// Включение реле S4X
#define RELAY_COM_RELAY_S4X       0x34
// Включение реле SR
#define RELAY_COM_RELAY_SR        0x35
// Включение реле SRC
#define RELAY_COM_RELAY_SRC       0x36

// Биты статуса
// Команда выполнена
#define RELAY_ST_OK               0x02

// Выделение байта из переменной
#define DATA_BYTE(_VAR_, _BIT_) (*(((irs_u8 *)&(_VAR_)) + ((_BIT_&0xF0) >> 4)))
// Выставление бита
#define DATA_BIT(_BIT_) (1 << (_BIT_&0x0F))
// Выставление инвертированного бита
#define DATA_IBIT(_BIT_) (0xFF^DATA_BIT(_BIT_))
// Проверка бита
#define DATA_TEST_BIT(_VAR_, _BIT_)\
  (DATA_BYTE(_VAR_, _BIT_)&DATA_BIT(_BIT_))
// Установка бита
#define DATA_SET_BIT(_VAR_, _BIT_, _VAL_)\
  ((_VAL_)?(DATA_BYTE(_VAR_, _BIT_) |= static_cast<irs_u8>(DATA_BIT(_BIT_))):\
    (DATA_BYTE(_VAR_, _BIT_) &= static_cast<irs_u8>(DATA_IBIT(_BIT_))))

// Структура сетевого массива
typedef struct _data_t {
  // Команда
  irs_i32 command;
  // Статус
  irs_i32 status;
  // Отладочная переменная
  irs_i32 debug;
} data_t;

// Размер сетевого массива в переменных
#define SIZE_OF_DATA (sizeof(data_t)/sizeof(irs_i32))


namespace u309m {

struct rele_ext_eth_data_t {
  irs::bit_data_t SYM_2V;
  irs::bit_data_t SYM_20V;
  irs::bit_data_t SYM_200V;
  irs::bit_data_t KZ_2V;
  irs::bit_data_t KZ_1A;
  irs::bit_data_t KZ_17A;
  irs::bit_data_t REL_220V;
  irs::bit_data_t SYM_OFF;
  irs::bit_data_t SYM_OFF_TEST;

  rele_ext_eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    SYM_2V.connect(ap_data, index, 0);
    SYM_20V.connect(ap_data, index, 1);
    SYM_200V.connect(ap_data, index, 2);
    KZ_2V.connect(ap_data, index, 3);
    KZ_1A.connect(ap_data, index, 4);
    KZ_17A.connect(ap_data, index, 5);
    REL_220V.connect(ap_data, index, 6);
    SYM_OFF.connect(ap_data, index, 7);
    index++;
    SYM_OFF_TEST.connect(ap_data, index, 0);
    index++;

    return index;
  }
}; // rele_ext_eth_data_t

enum supply_type_t{
  sup_200V,
  sup_20V,
  sup_2V,
  sup_1A,
  sup_17A
};

struct supply_comm_data_t
{
  irs::conn_data_t<irs_u8> supply_index;
  irs::conn_data_t<irs_u8> etalon_cell;
  irs::conn_data_t<irs_u8> calibrated_cell;
  irs::bit_data_t polarity_etalon;
  irs::bit_data_t polarity_calibrated;
  irs::bit_data_t apply;
  irs::bit_data_t on;
  irs::bit_data_t error;
  irs::bit_data_t reset;
  irs::bit_data_t debug;

  supply_comm_data_t(irs::mxdata_t *ap_data = IRS_NULL,
    irs_uarc a_index = 0, irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_start_index = 0)
  {
    irs_uarc index = a_start_index;

    index = supply_index.connect(ap_data, index);
    index = etalon_cell.connect(ap_data, index);
    index = calibrated_cell.connect(ap_data, index);
    polarity_etalon.connect(ap_data, index, 0);
    polarity_calibrated.connect(ap_data, index, 1);
    apply.connect(ap_data, index, 2);
    on.connect(ap_data, index, 3);
    error.connect(ap_data, index, 4);
    reset.connect(ap_data, index, 5);
    debug.connect(ap_data, index, 7);
    index++;

    return index;
  }
}; // supply_comm_data_t

struct meas_comm_data_t
{
  irs::conn_data_t<irs_u8> mode;
  irs::conn_data_t<irs_u8> etalon_cell;
  irs::conn_data_t<irs_u8> calibrated_cell;
  irs::bit_data_t load_resistor;
  irs::bit_data_t apply;
  irs::bit_data_t on;
  irs::bit_data_t error;
  irs::bit_data_t reset;
  irs::bit_data_t debug;

  meas_comm_data_t(irs::mxdata_t *ap_data = IRS_NULL,
    irs_uarc a_index = 0, irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_index = 0)
  {
    irs_uarc index = a_index;

    index = mode.connect(ap_data, index);
    index = etalon_cell.connect(ap_data, index);
    index = calibrated_cell.connect(ap_data, index);
    load_resistor.connect(ap_data, index, 0);
    apply.connect(ap_data, index, 1);
    on.connect(ap_data, index, 2);
    error.connect(ap_data, index, 3);
    reset.connect(ap_data, index, 4);
    debug.connect(ap_data, index, 7);
    index++;

    return index;
  }
}; // meas_comm_data_t

struct meas_comm_th_data_t
{
  irs::conn_data_t<float> th1_value;
  irs::conn_data_t<float> th2_value;
  irs::conn_data_t<float> th3_value;
  irs::conn_data_t<float> th4_value;
  irs::conn_data_t<float> th5_value;

  meas_comm_th_data_t(irs::mxdata_t *ap_data = IRS_NULL,
    irs_uarc a_index = 0, irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_index = 0)
  {
    irs_uarc index = a_index;

    index = th1_value.connect(ap_data, index);
    index = th2_value.connect(ap_data, index);
    index = th3_value.connect(ap_data, index);
    index = th4_value.connect(ap_data, index);
    index = th5_value.connect(ap_data, index);

    return index;
  }
}; // meas_comm_th_data_t

struct arm_adc_data_t {
  irs::conn_data_t<float> PTC_A;
  irs::conn_data_t<float> PTC_LC;
  irs::conn_data_t<float> TR_24V_TEST;
  irs::conn_data_t<float> IZM_3_3V_TEST;
  irs::conn_data_t<float> IZM_6V_TEST;
  irs::conn_data_t<float> IZM_1_2V_TEST;
  irs::conn_data_t<float> TEST_24V;
  irs::conn_data_t<float> TEST_5V;
  irs::conn_data_t<float> PTC_PWR;
  irs::conn_data_t<float> PTC_17A;
  irs::conn_data_t<float> internal_temp;

  arm_adc_data_t(irs::mxdata_t *ap_data = IRS_NULL,
    irs_uarc a_index = 0, irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_index = 0)
  {
    irs_uarc index = a_index;

    index = PTC_A.connect(ap_data, index);
    index = PTC_LC.connect(ap_data, index);
    index = TR_24V_TEST.connect(ap_data, index);
    index = IZM_3_3V_TEST.connect(ap_data, index);
    index = IZM_6V_TEST.connect(ap_data, index);
    index = IZM_1_2V_TEST.connect(ap_data, index);
    index = TEST_24V.connect(ap_data, index);
    index = TEST_5V.connect(ap_data, index);
    index = PTC_PWR.connect(ap_data, index);
    index = PTC_17A.connect(ap_data, index);
    index = internal_temp.connect(ap_data, index);

    return index;
  }
}; // arm_adc_data_t

struct temp_data_t {
  irs::conn_data_t<float> value;
  irs::conn_data_t<float> filtered_value;

  temp_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if(ap_size != IRS_NULL){
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    index = value.connect(ap_data, index);
    index = filtered_value.connect(ap_data, index);

    return index;
  }
};

struct dac_data_t {
  irs::conn_data_t<float> voltage_code;
  irs::conn_data_t<float> koef;

  dac_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if(ap_size != IRS_NULL){
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    index = voltage_code.connect(ap_data, index);
    index = koef.connect(ap_data, index);

    return index;
  }
};

struct adc_data_t {
  irs::conn_data_t<float> voltage_code;
  irs::conn_data_t<float> koef;

  adc_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if(ap_size != IRS_NULL){
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    index = voltage_code.connect(ap_data, index);
    index = koef.connect(ap_data, index);

    return index;
  }
};

struct tr_data_t {
  irs::conn_data_t<float> temperature_ref;
  irs::conn_data_t<float> temp_k;
  irs::conn_data_t<float> temp_ki;
  irs::conn_data_t<float> temp_kd;
  irs::conn_data_t<float> dac_value;
  irs::conn_data_t<float> temp_prop_koef;
  irs::conn_data_t<float> temp_time_const;
  irs::conn_data_t<irs_i32> int_val;

  tr_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if(ap_size != IRS_NULL){
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    index = temperature_ref.connect(ap_data, index);
    index = temp_k.connect(ap_data, index);
    index = temp_ki.connect(ap_data, index);
    index = temp_kd.connect(ap_data, index);
    index = dac_value.connect(ap_data, index);
    index = temp_prop_koef.connect(ap_data, index);
    index = temp_time_const.connect(ap_data, index);
    index = int_val.connect(ap_data, index);

    return index;
  }
};

struct supply_eth_data_t {
  irs::conn_data_t<irs_u16> resistance_code;
  dac_data_t prev_dac_data; // 8
  adc_data_t prev_adc_data; // 8
  dac_data_t fin_dac_data; // 8
  adc_data_t fin_adc_data; // 8
  tr_data_t base_tr_data; // 32
  temp_data_t base_temp_data; // 8
  tr_data_t aux_tr_data; // 32
  temp_data_t aux_temp_data; // 8

  supply_eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if(ap_size != IRS_NULL){
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    index = resistance_code.connect(ap_data, index);

    index = prev_dac_data.connect(ap_data, index);
    index = prev_adc_data.connect(ap_data, index);
    index = fin_dac_data.connect(ap_data, index);
    index = fin_adc_data.connect(ap_data, index);

    index = base_tr_data.connect(ap_data, index);
    index = base_temp_data.connect(ap_data, index);

    index = aux_tr_data.connect(ap_data, index);
    index = aux_temp_data.connect(ap_data, index);

    return index;
  }
}; // supply_eth_data_t

struct control_data_t
{
  irs::conn_data_t<irs_u32> alarm;
  irs::bit_data_t alarm_internal_th;
  irs::bit_data_t alarm_ptc_a;
  irs::bit_data_t alarm_ptc_lc;
  irs::bit_data_t alarm_ptc_pwr;
  irs::bit_data_t alarm_ptc_17A;
  irs::bit_data_t alarm_tr_24V;
  irs::bit_data_t alarm_24V;
  irs::bit_data_t alarm_5V;
  irs::bit_data_t alarm_izm_6V;
  irs::bit_data_t alarm_izm_3_3V;
  irs::bit_data_t alarm_izm_1_2V;
  irs::bit_data_t alarm_izm_th1;
  irs::bit_data_t alarm_izm_th2;
  irs::bit_data_t alarm_izm_th3;
  irs::bit_data_t alarm_izm_th4;
  irs::bit_data_t alarm_izm_th5;
  irs::bit_data_t alarm_200V_th_base;
  irs::bit_data_t alarm_200V_th_aux;
  irs::bit_data_t alarm_20V_th_base;
  irs::bit_data_t alarm_20V_th_aux;
  irs::bit_data_t alarm_2V_th_base;
  irs::bit_data_t alarm_2V_th_aux;
  irs::bit_data_t alarm_1A_th_base;
  irs::bit_data_t alarm_1A_th_aux;
  irs::bit_data_t alarm_17A_th_base;
  irs::bit_data_t alarm_17A_th_aux;
  irs::bit_data_t alarm_upper_level;

  irs::bit_data_t on;

  irs::conn_data_t<irs_u8> unlock;

  irs::bit_data_t thermo_200V_off;
  irs::bit_data_t meas_200V_off;
  irs::bit_data_t thermo_20V_off;
  irs::bit_data_t meas_20V_off;
  irs::bit_data_t thermo_2V_off;
  irs::bit_data_t meas_2V_off;
  irs::bit_data_t thermo_1A_off;
  irs::bit_data_t meas_1A_off;
  irs::bit_data_t thermo_17A_off;
  irs::bit_data_t meas_17A_off;

  irs::bit_data_t upper_level_check;
  irs::bit_data_t refresh_all_sources;
  irs::bit_data_t watchdog_reset_cause;
  irs::bit_data_t watchdog_test;
  irs::bit_data_t izm_th_spi_enable;
  irs::bit_data_t spi_enable;

  irs::conn_data_t<irs_u32> connect_counter;
  irs::conn_data_t<irs_u32> time;
  irs::conn_data_t<irs_u32> work_counter;

  control_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    alarm_internal_th.connect(ap_data, index, 0);
    alarm_ptc_a.connect(ap_data, index, 1);
    alarm_ptc_lc.connect(ap_data, index, 2);
    alarm_ptc_pwr.connect(ap_data, index, 3);
    alarm_ptc_17A.connect(ap_data, index, 4);
    alarm_tr_24V.connect(ap_data, index, 5);
    alarm_24V.connect(ap_data, index, 6);
    alarm_5V.connect(ap_data, index, 7);

    alarm_izm_6V.connect(ap_data, index + 1, 0);
    alarm_izm_3_3V.connect(ap_data, index + 1, 1);
    alarm_izm_1_2V.connect(ap_data, index + 1, 2);
    alarm_izm_th1.connect(ap_data, index + 1, 3);
    alarm_izm_th2.connect(ap_data, index + 1, 4);
    alarm_izm_th3.connect(ap_data, index + 1, 5);
    alarm_izm_th4.connect(ap_data, index + 1, 6);
    alarm_izm_th5.connect(ap_data, index + 1, 7);

    alarm_200V_th_base.connect(ap_data, index + 2, 0);
    alarm_200V_th_aux.connect(ap_data, index + 2, 1);
    alarm_20V_th_base.connect(ap_data, index + 2, 2);
    alarm_20V_th_aux.connect(ap_data, index + 2, 3);
    alarm_2V_th_base.connect(ap_data, index + 2, 4);
    alarm_2V_th_aux.connect(ap_data, index + 2, 5);
    alarm_1A_th_base.connect(ap_data, index + 2, 6);
    alarm_1A_th_aux.connect(ap_data, index + 2, 7);

    alarm_17A_th_base.connect(ap_data, index + 3, 0);
    alarm_17A_th_aux.connect(ap_data, index + 3, 1);
    alarm_upper_level.connect(ap_data, index + 3, 2);

    on.connect(ap_data, index + 3, 7);

    index = alarm.connect(ap_data, index);

    index = unlock.connect(ap_data, index);

    thermo_200V_off.connect(ap_data, index, 0);
    meas_200V_off.connect(ap_data, index, 1);
    thermo_20V_off.connect(ap_data, index, 2);
    meas_20V_off.connect(ap_data, index, 3);
    thermo_2V_off.connect(ap_data, index, 4);
    meas_2V_off.connect(ap_data, index, 5);
    thermo_1A_off.connect(ap_data, index, 6);
    meas_1A_off.connect(ap_data, index, 7);

    index++;

    thermo_17A_off.connect(ap_data, index, 0);
    meas_17A_off.connect(ap_data, index, 1);

    upper_level_check.connect(ap_data, index, 2);
    refresh_all_sources.connect(ap_data, index, 3);
    watchdog_reset_cause.connect(ap_data, index, 4);
    watchdog_test.connect(ap_data, index, 5);
    izm_th_spi_enable.connect(ap_data, index, 6);
    spi_enable.connect(ap_data, index, 6);

    index++;
    index++;

    index = connect_counter.connect(ap_data, index);
    index = time.connect(ap_data, index);
    index = work_counter.connect(ap_data, index);

    return index;
  }
}; // control_data_t

struct supply_add_data_t
{
  irs::bit_data_t* p_thermo_off;
  irs::bit_data_t* p_meas_off;
  
  supply_add_data_t():
    p_thermo_off(IRS_NULL),
    p_meas_off(IRS_NULL)
  {
  }
};

struct eth_data_t {
  // Биты (14 байт)
  irs::conn_data_t<irs_u8> ip_0;  //  1 byte
  irs::conn_data_t<irs_u8> ip_1;  //  1 byte
  irs::conn_data_t<irs_u8> ip_2;  //  1 byte
  irs::conn_data_t<irs_u8> ip_3;  //  1 byte
  rele_ext_eth_data_t rele_ext;   //  2 bytes
  supply_comm_data_t supply_comm; //  4 bytes
  meas_comm_data_t meas_comm;     //  4 bytes
  // Регистры
  meas_comm_th_data_t meas_comm_th;// 20 bytes
  arm_adc_data_t arm_adc;         //  44 bytes
  supply_eth_data_t supply_200V;  //  114 bytes
  supply_eth_data_t supply_20V;   //  114 bytes
  supply_eth_data_t supply_2V;    //  114 bytes
  supply_eth_data_t supply_1A;    //  114 bytes
  supply_eth_data_t supply_17A;   //  114 bytes
  control_data_t control;         //  20 bytes
  //---------------------------------------------
  //                          Итого:  668 байт

  eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }
  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    index = ip_0.connect(ap_data, index);
    index = ip_1.connect(ap_data, index);
    index = ip_2.connect(ap_data, index);
    index = ip_3.connect(ap_data, index);
    index = rele_ext.connect(ap_data, index);
    index = supply_comm.connect(ap_data, index);
    index = meas_comm.connect(ap_data, index);
    index = meas_comm_th.connect(ap_data, index);
    index = arm_adc.connect(ap_data, index);
    index = supply_200V.connect(ap_data, index);
    index = supply_20V.connect(ap_data, index);
    index = supply_2V.connect(ap_data, index);
    index = supply_1A.connect(ap_data, index);
    index = supply_17A.connect(ap_data, index);
    index = control.connect(ap_data, index);

    return index;
  }
};

} // namespace u309m

#endif // common_datah
