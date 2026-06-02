use std::sync::Mutex;

use crate::drivers::it930x::{CtrlMsgError, I2CCommRequest, I2CRequestType, IT930x};
use crate::drivers::itedtv_bus::BusOps;
use crate::drivers::px4_device::Tuner;
use crate::drivers::tc90522::{System, TunerError, TC90522};

const R850_NUM_REGS: usize = 0x30;

// C の init_regs 配列を Rust にコピー
pub const INIT_REGS: [u8; R850_NUM_REGS] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xca, 0xc0, 0x72, 0x50, 0x00, 0xe0, 0x00, 0x30,
    0x86, 0xbb, 0xf8, 0xb0, 0xd2, 0x81, 0xcd, 0x46, 0x37, 0x40, 0x89, 0x8c, 0x55, 0x95, 0x07, 0x23,
    0x21, 0xf1, 0x4c, 0x5f, 0xc4, 0x20, 0xa9, 0x6c, 0x53, 0xab, 0x5b, 0x46, 0xb3, 0x93, 0x6e, 0x41,
];

const SLEEP_REGS: [u8; R850_NUM_REGS] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x33, 0xee, 0xb9, 0xfe, 0x0f, 0xe1, 0x04, 0x30,
    0x86, 0xfb, 0xf8, 0xb0, 0xd2, 0x81, 0xcd, 0x46, 0x37, 0x44, 0x89, 0x8c, 0x55, 0x95, 0x07, 0x23,
    0x21, 0xf1, 0x4c, 0x5f, 0xc4, 0x20, 0xa9, 0xfc, 0x53, 0xab, 0x0b, 0x46, 0xb3, 0x93, 0x6e, 0x41,
];

const WAKEUP_REGS: [u8; R850_NUM_REGS] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xca, 0xe0, 0xf2, 0x7c, 0xc0, 0xe0, 0x00, 0x30,
    0x86, 0xbb, 0xf8, 0xb0, 0xd2, 0x81, 0xcd, 0x46, 0x37, 0x40, 0x89, 0x8c, 0x55, 0x95, 0x07, 0x23,
    0x21, 0xf1, 0x4c, 0x5f, 0xc4, 0x20, 0xa9, 0x6c, 0x53, 0xab, 0x5b, 0x46, 0xb3, 0x93, 0x6e, 0x41,
];

const IMR_CAL_REGS: [u8; R850_NUM_REGS] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x49, 0x3a, 0x90, 0x03, 0xc1, 0x61, 0x71,
    0x17, 0xf1, 0x18, 0x55, 0x30, 0x20, 0xf3, 0xed, 0x1f, 0x1c, 0x81, 0x13, 0x00, 0x80, 0x0a, 0x07,
    0x21, 0x71, 0x54, 0xf1, 0xf2, 0xa9, 0xbb, 0x0b, 0xa3, 0xf6, 0x0b, 0x44, 0x92, 0x17, 0xe6, 0x80,
];

const LPF_CAL_REGS: [u8; R850_NUM_REGS] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x49, 0x3f, 0x90, 0x13, 0xe1, 0x89, 0x7a,
    0x07, 0xf1, 0x9a, 0x50, 0x30, 0x20, 0xe1, 0x00, 0x00, 0x04, 0x81, 0x11, 0xef, 0xee, 0x17, 0x07,
    0x31, 0x71, 0x54, 0xb2, 0xee, 0xa9, 0xbb, 0x0b, 0xa3, 0x00, 0x0b, 0x44, 0x92, 0x1f, 0xe6, 0x80,
];

pub const LNA_ACC_GAIN: [u16; 32] = [
    0, 15, 26, 34, 50, 61, 75, 87, 101, 117, 130, 144, 154, 164, 176, 188, 199, 209, 220, 226, 233,
    232, 232, 232, 232, 247, 262, 280, 296, 311, 296, 308,
];

pub const RF_ACC_GAIN: [u16; 16] = [
    0, 15, 26, 34, 50, 61, 75, 87, 101, 117, 130, 144, 154, 164, 176, 188,
];

pub const MIXER_ACC_GAIN: [u16; 16] = [
    0, 0, 0, 0, 9, 22, 32, 44, 56, 68, 80, 90, 100, 100, 100, 100,
];

// px4_device.c の定義を、こっちでしか使わないので移動
// 地デジ用 (ISDB-T) 初期化パラメータ
// &[u8] にするために、値の前に `&` をつけ、配列リテラル `[...]` で囲む
const TC_INIT_T: [(u8, &'static [u8]); 10] = [
    (0xb0, &[0xa0]),
    (0xb2, &[0x3d]),
    (0xb3, &[0x25]),
    (0xb4, &[0x8b]),
    (0xb5, &[0x4b]),
    (0xb6, &[0x3f]),
    (0xb7, &[0xff]),
    (0xb8, &[0xc0]),
    (0x1f, &[0x00]),
    (0x75, &[0x00]),
];

// px4_device.c の定義を、こっちでしか使わないので移動
// デバイス全体の初期化用 (地デジ)
const TC_INIT_T0: [(u8, &'static [u8]); 2] = [(0x0e, &[0x77]), (0x0f, &[0x13])];

// 設定
#[derive(Debug, Clone, Copy)]
pub struct R850Config {
    pub xtal: u32,
    pub loop_through: bool,
    pub clock_out: bool,
    pub no_imr_calibration: bool,
    pub no_lpf_calibration: bool,
}

// システム定義
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum R850System {
    Undefined = 0,
    DvbT,
    DvbT2,
    DvbT2_1,
    DvbC,
    J83B,
    IsdbT,
    Dtmb,
    Atsc,
    Fm,
}

// 帯域幅
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum R850Bandwidth {
    B6M = 0,
    B7M,
    B8M,
}

// システム設定
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct R850SystemConfig {
    pub system: R850System,
    pub bandwidth: R850Bandwidth,
    pub if_freq: u32,
}

// IMR 構造体
#[derive(Debug, Clone, Copy)]
pub struct R850Imr {
    pub gain: u8,
    pub phase: u8,
    pub iqcap: u8,
    pub value: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum R850ImrDirection {
    Gain = 0,
    Phase = 1,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum R850Calibration {
    None = 0,
    IMR,
    LPF,
}

// 内部状態
#[derive(Debug)]
pub struct R850Priv {
    pub init: bool,
    pub chip: i32,
    pub xtal_pwr: u8,
    pub regs: [u8; R850_NUM_REGS],
    pub sleep: bool,
    pub sys: R850SystemConfig,
    pub mixer_mode: u8,
    pub mixer_amp_lpf_imr_cal: u8,
    pub imr_cal: [R850ImrCal; 2],
    pub sys_curr: R850SystemConfig,
}

#[derive(Debug)]
pub struct R850ImrCal {
    pub imr: [R850Imr; 5],
    pub done: bool,
    pub result: [bool; 5],
    pub mixer_amp_lpf: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LPFparams {
    pub code: u8,
    pub bandwidth: u8,
    pub lsb: u8,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct R850SystemParams {
    pub bandwidth: R850Bandwidth,
    pub if_freq: u32,
    pub filt_cal_if: u32,
    pub bw: u8,
    pub filt_ext_ena: u8,
    pub hpf_notch: u8,
    pub hpf_cor: u8,
    pub filt_comp: u8,
    pub img_gain: u8,
    pub agc_clk: u8,
    pub lpf: LPFparams,
}

#[derive(Debug, Clone, Copy)]
pub struct R850SystemFrequencyParams {
    pub if_freq: u32,
    pub rf_freq_min: u32,
    pub rf_freq_max: u32,
    pub lna_top: u8,
    pub lna_vtl_h: u8,
    pub lna_nrb_det: u8,
    pub lna_rf_dis_mode: u8,
    pub lna_rf_charge_cur: u8,
    pub lna_rf_dis_curr: u8,
    pub lna_dis_slow_fast: u8,
    pub rf_top: u8,
    pub rf_vtl_h: u8,
    pub rf_gain_limit: u8,
    pub rf_dis_slow_fast: u8,
    pub rf_lte_psg: u8,
    pub nrb_top: u8,
    pub nrb_bw_hpf: u8,
    pub nrb_bw_lpf: u8,
    pub mixer_top: u8,
    pub mixer_vth: u8,
    pub mixer_vtl: u8,
    pub mixer_amp_lpf: u8,
    pub mixer_gain_limit: u8,
    pub mixer_detbw_lpf: u8,
    pub mixer_filter_dis: u8,
    pub filter_top: u8,
    pub filter_vth: u8,
    pub filter_vtl: u8,
    pub filt_3th_lpf_cur: u8,
    pub filt_3th_lpf_gain: u8,
    pub bb_dis_curr: u8,
    pub bb_det_mode: u8,
    pub na_pwr_det: u8,
    pub enb_poly_gain: u8,
    pub img_nrb_adder: u8,
    pub hpf_comp: u8,
    pub fb_res_1st: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct R850LpfParams {
    pub code: u8,
    pub bandwidth: u8,
    pub lsb: u8,
}

// DVB-T/T2 Params [2][6]
const DVB_T_T2_PARAMS: [[R850SystemParams; 6]; 2] = [
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4570,
            filt_cal_if: 7550,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x08,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x01,
                bandwidth: 3,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B7M,
            if_freq: 4570,
            filt_cal_if: 7920,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0b,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 4570,
            filt_cal_if: 8450,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0c,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x01,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7920,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x06,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x06,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B7M,
            if_freq: 5000,
            filt_cal_if: 8450,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x09,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x00,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 5000,
            filt_cal_if: 8700,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0a,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x06,
                bandwidth: 0,
                lsb: 1,
            },
        },
    ],
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4570,
            filt_cal_if: 7550,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x08,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x01,
                bandwidth: 3,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B7M,
            if_freq: 4570,
            filt_cal_if: 7920,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0b,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 4570,
            filt_cal_if: 8450,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0c,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x01,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7920,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x06,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x06,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B7M,
            if_freq: 5000,
            filt_cal_if: 8450,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x09,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x00,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 5000,
            filt_cal_if: 8700,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0a,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x06,
                bandwidth: 0,
                lsb: 1,
            },
        },
    ],
];

// DVB-T2_1 Params [2][2]
const DVB_T2_1_PARAMS: [[R850SystemParams; 2]; 2] = [
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B7M,
            if_freq: 1900,
            filt_cal_if: 7920,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x08,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B7M,
            if_freq: 5000,
            filt_cal_if: 6000,
            bw: 2,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x01,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x0b,
                bandwidth: 3,
                lsb: 1,
            },
        },
    ],
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B7M,
            if_freq: 1900,
            filt_cal_if: 7920,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x08,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B7M,
            if_freq: 5000,
            filt_cal_if: 6000,
            bw: 2,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x01,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x0b,
                bandwidth: 3,
                lsb: 1,
            },
        },
    ],
];

// DVB-C Params [2][4]
const DVB_C_PARAMS: [[R850SystemParams; 4]; 2] = [
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5070,
            filt_cal_if: 8100,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x02,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 5070,
            filt_cal_if: 9550,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0b,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 0,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7780,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x06,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x01,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 5000,
            filt_cal_if: 9250,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0b,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x05,
                bandwidth: 0,
                lsb: 1,
            },
        },
    ],
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5070,
            filt_cal_if: 8100,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x02,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 5070,
            filt_cal_if: 9550,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0b,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 0,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7780,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x06,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x01,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 5000,
            filt_cal_if: 9250,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0b,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x05,
                bandwidth: 0,
                lsb: 1,
            },
        },
    ],
];

// J83B Params [2][2]
const J83B_PARAMS: [[R850SystemParams; 2]; 2] = [
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5070,
            filt_cal_if: 8100,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x03,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7550,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x05,
                bandwidth: 2,
                lsb: 1,
            },
        },
    ],
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5070,
            filt_cal_if: 8100,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x03,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7550,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x05,
                bandwidth: 2,
                lsb: 1,
            },
        },
    ],
];

// ISDB-T Params [2][3]
const ISDB_T_PARAMS: [[R850SystemParams; 3]; 2] = [
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4063,
            filt_cal_if: 7070,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x08,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x02,
                bandwidth: 3,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4570,
            filt_cal_if: 7400,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x08,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7780,
            bw: 1,
            filt_ext_ena: 1,
            hpf_notch: 0,
            hpf_cor: 0x03,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x05,
                bandwidth: 2,
                lsb: 0,
            },
        },
    ],
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4063,
            filt_cal_if: 7070,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0a,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x02,
                bandwidth: 3,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4570,
            filt_cal_if: 7400,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x08,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x08,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7780,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x03,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x05,
                bandwidth: 2,
                lsb: 0,
            },
        },
    ],
];

// DTMB Params [2][4]
const DTMB_PARAMS: [[R850SystemParams; 4]; 2] = [
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4500,
            filt_cal_if: 7200,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x08,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x02,
                bandwidth: 3,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 4570,
            filt_cal_if: 8450,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0c,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x00,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 8100,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x06,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 5000,
            filt_cal_if: 8800,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0b,
            filt_comp: 2,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x05,
                bandwidth: 0,
                lsb: 1,
            },
        },
    ],
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4500,
            filt_cal_if: 7200,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x08,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x02,
                bandwidth: 3,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 4570,
            filt_cal_if: 8450,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0c,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x00,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 8100,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x06,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 2,
                lsb: 1,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B8M,
            if_freq: 5000,
            filt_cal_if: 8800,
            bw: 0,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x0b,
            filt_comp: 2,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x05,
                bandwidth: 0,
                lsb: 1,
            },
        },
    ],
];

// ATSC Params [2][2]
const ATSC_PARAMS: [[R850SystemParams; 2]; 2] = [
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5070,
            filt_cal_if: 8050,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x03,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7920,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 0,
            agc_clk: 0,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 2,
                lsb: 0,
            },
        },
    ],
    [
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5070,
            filt_cal_if: 8050,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x03,
                bandwidth: 2,
                lsb: 0,
            },
        },
        R850SystemParams {
            bandwidth: R850Bandwidth::B6M,
            if_freq: 5000,
            filt_cal_if: 7920,
            bw: 1,
            filt_ext_ena: 0,
            hpf_notch: 0,
            hpf_cor: 0x05,
            filt_comp: 1,
            img_gain: 3,
            agc_clk: 1,
            lpf: LPFparams {
                code: 0x04,
                bandwidth: 2,
                lsb: 0,
            },
        },
    ],
];

/// マスターテーブル
/// 各システムの配列のスライスを参照する形式にしています。
pub const SYS_PARAMS: &[&[&[R850SystemParams]]] = &[
    &[&[], &[]],                                 // 0: UNDEFINED
    &[&DVB_T_T2_PARAMS[0], &DVB_T_T2_PARAMS[1]], // 1: DVB_T
    &[&DVB_T_T2_PARAMS[0], &DVB_T_T2_PARAMS[1]], // 2: DVB_T2
    &[&DVB_T2_1_PARAMS[0], &DVB_T2_1_PARAMS[1]], // 3: DVB_T2_1
    &[&DVB_C_PARAMS[0], &DVB_C_PARAMS[1]],       // 4: DVB_C
    &[&J83B_PARAMS[0], &J83B_PARAMS[1]],         // 5: J83B
    &[&ISDB_T_PARAMS[0], &ISDB_T_PARAMS[1]],     // 6: IsdbT
    &[&DTMB_PARAMS[0], &DTMB_PARAMS[1]],         // 7: DTMB
    &[&ATSC_PARAMS[0], &ATSC_PARAMS[1]],         // 8: ATSC
    &[&[], &[]],                                 // 9: FM
];

pub const DVB_T_T2_FREQ_PARAMS: [R850SystemFrequencyParams; 4] = [
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 340000,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x5a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 5,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 9,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 2,
        filter_top: 4,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 2,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 662001,
        rf_freq_max: 670000,
        lna_top: 4,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 4,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x5a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 4,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 9,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 2,
        filter_top: 4,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 2,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 782001,
        rf_freq_max: 790000,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 2,
        lna_rf_charge_cur: 0,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x5a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 4,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 9,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 2,
        filter_top: 4,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 2,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 0,
        lna_top: 4,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x5a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 4,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 9,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 2,
        filter_top: 4,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 2,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
];

pub const DVB_C_FREQ_PARAMS: [R850SystemFrequencyParams; 2] = [
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 660000,
        lna_top: 4,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 0,
        nrb_top: 5,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 12,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 2,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 2,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 0,
        lna_top: 4,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 3,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 0,
        nrb_top: 5,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 12,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 2,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 1,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
];

pub const J83B_FREQ_PARAMS: [R850SystemFrequencyParams; 3] = [
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 335000,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 0,
        nrb_top: 5,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 0,
        mixer_top: 12,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 2,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 2,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 340001,
        rf_freq_max: 660000,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 0,
        nrb_top: 5,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 0,
        mixer_top: 12,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 2,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 2,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 0,
        lna_top: 4,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 3,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 0,
        nrb_top: 5,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 0,
        mixer_top: 12,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 2,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 1,
        hpf_comp: 1,
        fb_res_1st: 1,
    },
];

pub const ISDB_T_FREQ_PARAMS: [R850SystemFrequencyParams; 10] = [
    /* ISDB-T 4063 */
    R850SystemFrequencyParams {
        if_freq: 4063,
        rf_freq_min: 0,
        rf_freq_max: 340000,
        lna_top: 5,
        lna_vtl_h: 0x6b,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 5,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 12,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 15,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 2,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 4063,
        rf_freq_min: 470000,
        rf_freq_max: 487999,
        lna_top: 6,
        lna_vtl_h: 0x8c,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 5,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 3,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 14,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 3,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 4063,
        rf_freq_min: 680000,
        rf_freq_max: 691999,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 2,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x07,
        rf_top: 6,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x04,
        rf_lte_psg: 1,
        nrb_top: 3,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 14,
        mixer_vth: 0x09,
        mixer_vtl: 0x05,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 0,
        enb_poly_gain: 1,
        img_nrb_adder: 3,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 4063,
        rf_freq_min: 692000,
        rf_freq_max: 697999,
        lna_top: 5,
        lna_vtl_h: 0x5b,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 2,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x07,
        rf_top: 6,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x04,
        rf_lte_psg: 1,
        nrb_top: 10,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 3,
        mixer_top: 12,
        mixer_vth: 0x09,
        mixer_vtl: 0x05,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 0,
        enb_poly_gain: 1,
        img_nrb_adder: 2,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 4063,
        rf_freq_min: 0,
        rf_freq_max: 0,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 6,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 3,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 14,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 3,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    /* ISDB-T other */
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 340000,
        lna_top: 5,
        lna_vtl_h: 0x6b,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 5,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 12,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 15,
        mixer_vth: 0x0b,
        mixer_vtl: 0x06,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 2,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 470000,
        rf_freq_max: 487999,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 2,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x07,
        rf_top: 6,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x04,
        rf_lte_psg: 1,
        nrb_top: 3,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 14,
        mixer_vth: 0x09,
        mixer_vtl: 0x05,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 0,
        enb_poly_gain: 1,
        img_nrb_adder: 3,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 680000,
        rf_freq_max: 691999,
        lna_top: 5,
        lna_vtl_h: 0x5b,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 2,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x07,
        rf_top: 6,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x04,
        rf_lte_psg: 1,
        nrb_top: 10,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 3,
        mixer_top: 12,
        mixer_vth: 0x09,
        mixer_vtl: 0x05,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 0,
        enb_poly_gain: 1,
        img_nrb_adder: 2,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 692000,
        rf_freq_max: 697999,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 6,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 3,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 14,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 3,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 0,
        lna_top: 5,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 6,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 3,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 14,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 3,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 0,
        filter_top: 12,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 1,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 1,
        img_nrb_adder: 3,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
];

pub const DTMB_FREQ_PARAMS: [R850SystemFrequencyParams; 3] = [
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 100000,
        lna_top: 4,
        lna_vtl_h: 0x6b,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 10,
        nrb_bw_hpf: 3,
        nrb_bw_lpf: 3,
        mixer_top: 9,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 1,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 2,
        filter_top: 4,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 0,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 1,
        hpf_comp: 0,
        fb_res_1st: 0,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 340000,
        lna_top: 4,
        lna_vtl_h: 0x6b,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 10,
        nrb_bw_hpf: 0,
        nrb_bw_lpf: 2,
        mixer_top: 9,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 1,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 2,
        filter_top: 4,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 0,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 1,
        hpf_comp: 0,
        fb_res_1st: 0,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 0,
        lna_top: 4,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 4,
        rf_vtl_h: 0x4a,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 6,
        nrb_bw_hpf: 3,
        nrb_bw_lpf: 2,
        mixer_top: 9,
        mixer_vth: 0x09,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 4,
        mixer_gain_limit: 1,
        mixer_detbw_lpf: 0,
        mixer_filter_dis: 2,
        filter_top: 4,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 0,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 0,
        hpf_comp: 0,
        fb_res_1st: 0,
    },
];

pub const ATSC_FREQ_PARAMS: [R850SystemFrequencyParams; 2] = [
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 340000,
        lna_top: 6,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 5,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 12,
        nrb_bw_hpf: 2,
        nrb_bw_lpf: 2,
        mixer_top: 12,
        mixer_vth: 0x0b,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 2,
        mixer_detbw_lpf: 1,
        mixer_filter_dis: 2,
        filter_top: 6,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 0,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 1,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
    R850SystemFrequencyParams {
        if_freq: 0,
        rf_freq_min: 0,
        rf_freq_max: 0,
        lna_top: 6,
        lna_vtl_h: 0x5a,
        lna_nrb_det: 0,
        lna_rf_dis_mode: 1,
        lna_rf_charge_cur: 1,
        lna_rf_dis_curr: 1,
        lna_dis_slow_fast: 0x05,
        rf_top: 5,
        rf_vtl_h: 0x6b,
        rf_gain_limit: 0,
        rf_dis_slow_fast: 0x05,
        rf_lte_psg: 1,
        nrb_top: 12,
        nrb_bw_hpf: 2,
        nrb_bw_lpf: 2,
        mixer_top: 12,
        mixer_vth: 0x0b,
        mixer_vtl: 0x04,
        mixer_amp_lpf: 7,
        mixer_gain_limit: 2,
        mixer_detbw_lpf: 1,
        mixer_filter_dis: 2,
        filter_top: 6,
        filter_vth: 0x09,
        filter_vtl: 0x04,
        filt_3th_lpf_cur: 1,
        filt_3th_lpf_gain: 3,
        bb_dis_curr: 0,
        bb_det_mode: 0,
        na_pwr_det: 1,
        enb_poly_gain: 0,
        img_nrb_adder: 1,
        hpf_comp: 2,
        fb_res_1st: 1,
    },
];

/// 周波数パラメータのマスターテーブル
/// システムごとのスライスを保持します
pub const SYS_FREQ_PARAMS: &[&[R850SystemFrequencyParams]] = &[
    &[],                   // 0: UNDEFINED
    &DVB_T_T2_FREQ_PARAMS, // 1: DVB_T
    &DVB_T_T2_FREQ_PARAMS, // 2: DVB_T2
    &DVB_T_T2_FREQ_PARAMS, // 3: DVB_T2_1
    &DVB_C_FREQ_PARAMS,    // 4: DVB_C
    &J83B_FREQ_PARAMS,     // 5: J83B
    &ISDB_T_FREQ_PARAMS,   // 6: IsdbT
    &DTMB_FREQ_PARAMS,     // 7: DTMB
    &ATSC_FREQ_PARAMS,     // 8: ATSC
    &[],                   // 9: FM
];

pub struct R850<'a, B: BusOps> {
    tc90522: TC90522<'a, B>,

    // 設定パラメータ
    //pub xtal: u32,
    //pub loop_through: bool,
    //pub clock_out: bool,
    //pub no_imr_calibration: bool,
    //pub no_lpf_calibration: bool,
    pub config: R850Config,

    pub i2c_addr: u8,
    priv_: Mutex<R850Priv>,
}

impl<'a, B: BusOps> R850<'a, B> {
    // bit反転
    fn reverse_bit(val: u8) -> u8 {
        let mut t = val;

        t = ((t & 0x55) << 1) | ((t & 0xAA) >> 1);
        t = ((t & 0x33) << 2) | ((t & 0xCC) >> 2);
        ((t & 0x0F) << 4) | ((t & 0xF0) >> 4)
    }

    // レジスタ読み取り
    fn read_regs(&self, reg: u8, buf: &mut [u8]) -> Result<(), CtrlMsgError> {
        if (buf.len() == 0) || (buf.len() > (R850_NUM_REGS - reg as usize)) {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut write_buf = [0];
        let mut read_buf = vec![0u8; reg as usize + buf.len()];

        let mut reqs = [
            I2CCommRequest {
                addr: self.i2c_addr,
                data: &mut write_buf,
                req: I2CRequestType::Write,
            },
            I2CCommRequest {
                addr: self.i2c_addr,
                data: &mut read_buf,
                req: I2CRequestType::Read,
            },
        ];

        self.tc90522.i2c_master_request(&mut reqs)?;

        // ここで buf へ値を出す
        // 逆イテレータで reg のサイズ前まで取りつつ、reverse_bit()
        for i in 0..buf.len() {
            buf[i] = Self::reverse_bit(read_buf[reg as usize + i]);
        }

        Ok(())
    }

    // レジスタ書き込み
    fn write_regs(&self, reg: u8, buf: &[u8]) -> Result<(), CtrlMsgError> {
        if (buf.len() == 0) || (buf.len() > (R850_NUM_REGS - reg as usize)) {
            return Err(CtrlMsgError::InvalidLength);
        }

        let mut wbuf = Vec::with_capacity(1 + buf.len());
        wbuf.push(reg);
        wbuf.extend_from_slice(buf);

        let mut reqs = [I2CCommRequest {
            addr: self.i2c_addr,
            data: &mut wbuf,
            req: I2CRequestType::Write,
        }];

        self.tc90522.i2c_master_request(&mut reqs)
    }

    // メモ: 初期値(デフォルト値)に戻すイメージ
    fn init_regs(&self, priv_data: &mut R850Priv) {
        //let mut priv_data = self.priv_.lock().unwrap();
        priv_data.regs.copy_from_slice(&INIT_REGS);
    }

    // クリスタル振幅（XTAL CAP）の設定
    // すでにロックを取得済みの状態で呼び出す内部メソッド
    // 水晶発振器（XTAL）のキャパシタンス（発振の振幅や安定性に関わる容量）をレジスタに設定します。
    fn set_xtal_cap(&self, cap: u8, priv_data: &mut R850Priv) {
        let mut c = cap;
        let mut g = false;

        // capが0x1f(31)を超える場合の特殊処理
        if c > 0x1f {
            c -= 10;
            g = true;
        }

        // レジスタ 0x21 の更新:
        // bit 2-6 に c の下位5ビット、bit 7 に g のフラグを設定
        priv_data.regs[0x21] =
            (priv_data.regs[0x21] & 0x07) | ((c << 2) & 0x78) | (if g { 0x80 } else { 0x00 });

        // レジスタ 0x22 の更新:
        // bit 3 に c の特定ビットを反映
        priv_data.regs[0x22] = (priv_data.regs[0x22] & 0xf7) | ((c << 3) & 0x08);
    }

    // PLL（周波数）を設定する
    // すでにロックを取得済みの状態で呼び出す内部メソッド
    // ローカルオシレータ周波数（lo_freq）と中間周波数（if_freq）に基づき、PLLの分周比やSDM値を計算し、レジスタに適用する。
    // ISDB-Tの特定周波数帯では特殊なクロック割り算の補正を行う。
    fn set_pll(
        &self,
        lo_freq: u32,
        if_freq: u32,
        sys: R850System,
        priv_data: &mut R850Priv,
    ) -> Result<(), CtrlMsgError> {
        let mut xtal = self.config.xtal; // configから取得
        let mut vco_min = 2200000;

        // チップタイプによるVCO最小値の補正
        if priv_data.chip == 0 {
            vco_min += 70000;
        }

        let vco_max = vco_min * 2;
        let mut mix_div: u8 = 2;
        let mut vco_freq = lo_freq * mix_div as u32;

        // レジスタの基本ビット操作
        priv_data.regs[0x20] &= 0xfc;
        priv_data.regs[0x2e] |= 0x40;
        priv_data.regs[0x0c] &= 0x3c;
        priv_data.regs[0x09] &= 0xf9;
        priv_data.regs[0x22] &= 0x3f;
        priv_data.regs[0x0b] &= 0xc3;
        priv_data.regs[0x0b] |= 0x10;
        priv_data.regs[0x25] &= 0xef;
        priv_data.regs[0x25] |= 0x20;

        // xtal_pwr に基づく係数 b の算出
        let b = if lo_freq < 100000 {
            if priv_data.xtal_pwr > 1 {
                3 - priv_data.xtal_pwr
            } else {
                2
            }
        } else if lo_freq < 130000 {
            if priv_data.xtal_pwr > 2 {
                3 - priv_data.xtal_pwr
            } else {
                1
            }
        } else {
            0
        };

        //
        self.set_xtal_cap(0x27, priv_data);

        priv_data.regs[0x22] &= 0xcf;
        priv_data.regs[0x22] |= (b << 4) & 0x30;

        let div_judge = (lo_freq + if_freq) / 1000 / 12;

        priv_data.regs[0x1e] &= 0x1f;
        priv_data.regs[0x25] &= 0xfd;

        match div_judge {
            4 | 10 | 22 | 24 | 28 => priv_data.regs[0x25] |= 0x02,
            _ => priv_data.regs[0x25] |= 0x00,
        }

        if priv_data.chip != 0 {
            priv_data.regs[0x2f] &= 0xfd;
        } else {
            priv_data.regs[0x2f] &= 0xfc;
        }

        // VCO周波数の範囲に収まるまで mix_div を調整
        let mut div = 0;
        while div < 6 {
            if vco_min <= vco_freq && vco_freq < vco_max {
                break;
            }
            mix_div *= 2;
            vco_freq = lo_freq * mix_div as u32;
            div += 1;
        }

        let mut xtal_div = 0;
        priv_data.regs[0x22] &= 0xfc;
        if sys != R850System::Undefined {
            if lo_freq < 380500 {
                if (div_judge & 1) == 0 {
                    xtal /= 2;
                    priv_data.regs[0x22] |= 0x02;
                    xtal_div = 1;
                }
            } else if (478000..=481999).contains(&(lo_freq + if_freq)) && sys == R850System::IsdbT {
                // ((lo_freq + if_freq - 478000) < 4000 && sys == R850_SYSTEM_IsdbT)
                xtal /= 4;
                priv_data.regs[0x22] |= 0x03;
                xtal_div = 3;
            }
        }

        priv_data.regs[0x0b] &= 0xfe;
        priv_data.regs[0x2d] &= 0xf3;
        match mix_div {
            8 => priv_data.regs[0x2d] |= 0x04,
            16 => priv_data.regs[0x2d] |= 0x08,
            m if m >= 32 => priv_data.regs[0x2d] |= 0x0c,
            _ => {}
        }

        priv_data.regs[0x2e] &= 0xfc;
        priv_data.regs[0x20] &= 0xec;
        if mix_div == 2 || mix_div == 4 {
            priv_data.regs[0x2e] |= 0x01;
        } else {
            priv_data.regs[0x2e] |= 0x02;
            priv_data.regs[0x20] |= 0x01;
        }

        priv_data.regs[0x11] &= 0x7f;
        if mix_div == 8 {
            priv_data.regs[0x11] |= 0x80;
        }

        priv_data.regs[0x1e] &= 0xe3;
        priv_data.regs[0x1e] |= (div << 2) & 0x1c;

        // 分周比とSDMの計算
        let nint = (vco_freq / 2) / xtal;
        let mut vco_fra = vco_freq - (xtal * 2 * nint);

        // vco_fra の微調整
        if vco_fra < (xtal / 64) {
            vco_fra = 0;
        } else if vco_fra > (xtal * 127 / 64) {
            vco_fra = 0;
            // nint++ は Rust では明示的に変数を更新
        } else if vco_fra > (xtal * 127 / 128) && xtal > vco_fra {
            vco_fra = xtal * 127 / 128;
        } else if xtal < vco_fra && vco_fra < (xtal * 129 / 128) {
            vco_fra = xtal * 129 / 128;
        }

        // nint の補正（nint++ 相当）
        let final_nint = if vco_freq - (xtal * 2 * nint) > (xtal * 127 / 64) {
            nint + 1
        } else {
            nint
        };

        let ni = (final_nint - 13) / 4;
        let si = final_nint - 13 - (ni * 4);

        priv_data.regs[0x1b] &= 0x80;
        priv_data.regs[0x1b] |= ni as u8 & 0x7f;

        priv_data.regs[0x1e] &= 0xfc;
        priv_data.regs[0x1e] |= si as u8 & 0x03;

        priv_data.regs[0x20] &= 0x3f;

        // SDM の計算ループ
        let mut nsdm: u32 = 2;
        let mut sdm: u32 = 0;
        while vco_fra > 1 {
            if (xtal * 2 / nsdm) < vco_fra {
                vco_fra -= (xtal * 2) / nsdm;
                sdm += 0x8000 / (nsdm / 2);

                if (nsdm & 0x8000) != 0 {
                    break;
                }
            }
            nsdm += nsdm;
        }

        priv_data.regs[0x1c] = (sdm & 0xff) as u8;
        priv_data.regs[0x1d] = ((sdm >> 8) & 0xff) as u8;

        // I2C 書き込み（レジスタ 0x08 から 0x28 バイト分）
        self.write_regs(0x08, &priv_data.regs[0x08..0x30])?;

        // ウェイト処理
        match xtal_div {
            0 => std::thread::sleep(std::time::Duration::from_millis(10)),
            1 | 2 => std::thread::sleep(std::time::Duration::from_millis(20)),
            _ => std::thread::sleep(std::time::Duration::from_millis(40)),
        }

        if priv_data.chip == 0 {
            priv_data.regs[0x2f] &= 0xfc;
        }
        priv_data.regs[0x2f] |= 0x02;

        // 最後に 0x2f レジスタを書き込み
        self.write_regs(0x2f, &priv_data.regs[0x2f..0x30])
    }

    // MUX（各部フィルターやIMR設定）を更新する
    // すでにロックを取得済みの状態で呼び出す内部メソッド
    // 周波数帯域や放送規格（System）に合わせて、内部の各種フィルター（HPF, BPF, LPF Notch/Cap, Polyphaseなど）の経路や定数を決定し、レジスタに適用する。
    fn set_mux(
        &self,
        _rf_freq: u32, // Cコードで未使用のため先頭にアンダースコア
        lo_freq: u32,
        sys: R850System,
        priv_data: &mut R850Priv,
    ) {
        // 1. IMR インデックスの決定
        let imr_idx = if lo_freq < 170000 {
            0
        } else if lo_freq < 240000 {
            4
        } else if lo_freq < 400000 {
            1
        } else if lo_freq < 760000 {
            2
        } else {
            3
        };

        // 2. TF HPF/BPF の決定
        let tf_hpf_bpf = if lo_freq < 580000 {
            7
        } else if lo_freq < 660000 {
            1
        } else if lo_freq < 780000 {
            6
        } else if lo_freq < 900000 {
            4
        } else {
            0
        };

        // 3. RF Polyphase filter の決定
        let rf_poly = if lo_freq < 133000 {
            2
        } else if lo_freq < 221000 {
            1
        } else if lo_freq < 760000 {
            0
        } else {
            3
        };

        // 4. TF HPF CNR の決定
        let tf_hpf_cnr = if lo_freq < 480000 {
            3
        } else if lo_freq < 550000 {
            2
        } else if lo_freq < 700000 {
            1
        } else {
            0
        };

        // 5. LPF Notch / Cap の決定 (放送方式による分岐)
        let (lpf_notch, lpf_cap) = if sys == R850System::DvbC || sys == R850System::J83B {
            if lo_freq < 77000 {
                (10, 15)
            } else if lo_freq < 85000 {
                (4, 15)
            } else if lo_freq < 115000 {
                (3, 13)
            } else if lo_freq < 125000 {
                (1, 11)
            } else if lo_freq < 141000 {
                (0, 9)
            } else if lo_freq < 157000 {
                (0, 8)
            } else if lo_freq < 181000 {
                (0, 6)
            } else if lo_freq < 205000 {
                (0, 3)
            } else {
                (0, 0)
            }
        } else {
            if lo_freq < 73000 {
                (10, 8)
            } else if lo_freq < 81000 {
                (4, 8)
            } else if lo_freq < 89000 {
                (3, 8)
            } else if lo_freq < 121000 {
                (1, 6)
            } else if lo_freq < 145000 {
                (0, 4)
            } else if lo_freq < 153000 {
                (0, 3)
            } else if lo_freq < 177000 {
                (0, 2)
            } else if lo_freq < 201000 {
                (0, 1)
            } else {
                (0, 0)
            }
        };

        // 6. Diplexer の決定
        let tf_diplexer = if lo_freq < 330000 { 2 } else { 0 };

        // 7. IMR パラメータの決定 (キャリブレーション済みならその値を使用)
        let mixer_mode = priv_data.mixer_mode as usize;
        let (imr_gain, imr_phase, imr_iqcap) = if priv_data.imr_cal[mixer_mode].done
            && priv_data.imr_cal[mixer_mode].result[imr_idx as usize]
        {
            let imr = &priv_data.imr_cal[mixer_mode].imr[imr_idx as usize];
            (imr.gain, imr.phase, imr.iqcap)
        } else if sys != R850System::Undefined {
            (0x02, 0x00, 0x00)
        } else {
            (0x00, 0x00, 0x00)
        };

        // 8. レジスタキャッシュへの反映
        priv_data.regs[0x0e] =
            (priv_data.regs[0x0e] & 0x03) | ((tf_diplexer << 2) & 0x0c) | ((lpf_cap << 4) & 0xf0);

        priv_data.regs[0x0f] = (priv_data.regs[0x0f] & 0xf0) | (lpf_notch & 0x0f);

        priv_data.regs[0x10] =
            (priv_data.regs[0x10] & 0xe0) | ((tf_hpf_cnr << 3) & 0x18) | (tf_hpf_bpf & 0x07);

        priv_data.regs[0x12] = (priv_data.regs[0x12] & 0xfc) | (rf_poly & 0x03);

        priv_data.regs[0x14] = (priv_data.regs[0x14] & 0xd0) | (imr_gain & 0x2f);

        priv_data.regs[0x15] =
            (priv_data.regs[0x15] & 0x10) | (imr_phase & 0x2f) | ((imr_iqcap << 6) & 0xc0);
    }

    // ADCの値を読み取る
    // キャリブレーション時の信号誤差の測定に使われる、らしい。
    fn read_adc_value(&self) -> Result<u8, TunerError> {
        // 2ミリ秒待機 (ADCの変換完了を待つ)
        std::thread::sleep(std::time::Duration::from_millis(2));

        let mut tmp = [0u8; 1];
        // 0x01 レジスタから 1バイト読み取り
        self.read_regs(0x01, &mut tmp)?;

        // 下位 6ビット (0x3f) が有効な値
        Ok(tmp[0] & 0x3f)
    }

    // IMR (Image Rejection Ratio / イメージ除去比？) のゲイン/フェーズの誤差方向をチェックする
    // すでにロックを取得済みの状態で呼び出す内部メソッド
    // IMR最適化のため、十字方向探索アルゴリズムによる周辺探索を行い、ADCの誤差読み取りが最小になるIQゲイン、IQフェーズ、IQキャパシタの組み合わせを自動探索する、らしい。
    fn imr_check_iq_cross(
        &self,
        priv_data: &mut R850Priv,
    ) -> Result<(R850Imr, R850ImrDirection), TunerError> {
        // 十字探索用のパラメータテーブル (gain, phase)
        // Cコードの cross[9] を初期値を含めて再現
        let cross = [
            (0x00, 0x00),        // index 0
            (0x00, 0x01),        // index 1
            (0x00, 0x20 | 0x01), // index 2
            (0x01, 0x00),        // index 3
            (0x20 | 0x01, 0x00), // index 4
            (0x00, 0x02),        // index 5
            (0x00, 0x20 | 0x02), // index 6
            (0x02, 0x00),        // index 7
            (0x20 | 0x02, 0x00), // index 8
        ];

        let mut imr_best = R850Imr {
            gain: 0,
            phase: 0,
            iqcap: 0,
            value: 0xff, // 最小値を探すため最大値で初期化
        };

        for (c_gain, c_phase) in cross {
            // レジスタ 0x14 (Gain) の更新
            priv_data.regs[0x14] = (priv_data.regs[0x14] & 0xd0) | (c_gain & 0x2f);
            // レジスタ 0x15 (Phase) の更新
            priv_data.regs[0x15] = (priv_data.regs[0x15] & 0xd0) | (c_phase & 0x2f);

            // 0x14 から 2バイト (0x14, 0x15) 書き込み
            self.write_regs(0x14, &priv_data.regs[0x14..0x16])?;

            // ADC値を読み取り (前回移植したメソッド)
            let tmp = self.read_adc_value()?;

            // より低い値（誤差が少ないポイント）が見つかったら更新
            if imr_best.value > tmp {
                imr_best.gain = c_gain;
                imr_best.phase = c_phase;
                imr_best.value = tmp;
            }
        }

        // phase が設定されていれば Phase 方向、そうでなければ Gain 方向と判定
        let direction = if imr_best.phase != 0 {
            R850ImrDirection::Phase
        } else {
            R850ImrDirection::Gain
        };

        Ok((imr_best, direction))
    }

    // IMRのゲインまたはフェーズの周辺探索（3点または5点）
    // すでにロックを取得済みの状態で呼び出す内部メソッド
    // ツリー状の探索アルゴリズムを用いた周辺探索、らしい。
    fn imr_check_iq_tree(
        &self,
        imr: R850Imr,
        direction: R850ImrDirection,
        num: usize,
        priv_data: &mut R850Priv,
    ) -> Result<R850Imr, TunerError> {
        // エラーチェック
        if num != 3 && num != 5 {
            return Err(TunerError::InvalidArgument);
        }

        let reg_idx: usize;
        let mut val = [0u8; 5];
        //let mut imr_tmp = *imr;

        let mut imr_tmp = R850Imr {
            gain: 0,
            phase: 0,
            iqcap: 0,
            value: 0xff, // 最小値を探すため最大値で初期化
        };

        // 探索方向に応じたレジスタと初期値の設定
        match direction {
            R850ImrDirection::Gain => {
                reg_idx = 0x14; // ゲインレジスタ
                val[0] = imr.gain;
                // 反対側（フェーズ）のレジスタを現在の値で固定
                priv_data.regs[0x15] = (priv_data.regs[0x15] & 0xd0) | (imr.phase & 0x2f);
                imr_tmp.phase = imr.phase;
            }
            R850ImrDirection::Phase => {
                reg_idx = 0x15; // フェーズレジスタ
                val[0] = imr.phase;
                // 反対側（ゲイン）のレジスタを現在の値で固定
                priv_data.regs[0x14] = (priv_data.regs[0x14] & 0xd0) | (imr.gain & 0x2f);
                imr_tmp.gain = imr.gain;
            }
        }

        // 探索候補値（val配列）の計算
        val[1] = val[0].wrapping_add(1);

        if num == 3 {
            if (val[0] & 0x0f) == 0 {
                val[2] = (val[0] ^ 0x20).wrapping_add(1);
            } else {
                val[2] = val[0].wrapping_sub(1);
            }
        } else {
            // num == 5
            val[2] = val[0].wrapping_add(2);
            match val[0] & 0x0f {
                0 => {
                    val[3] = (val[0] ^ 0x20).wrapping_add(1);
                    val[4] = val[3].wrapping_add(1);
                }
                1 => {
                    val[3] = val[0].wrapping_sub(1);
                    val[4] = (val[3] ^ 0x20).wrapping_add(1);
                }
                _ => {
                    val[3] = val[0].wrapping_sub(1);
                    val[4] = val[3].wrapping_sub(1);
                }
            }
        }

        // 候補値を順番に試す
        for i in 0..num {
            // ターゲットレジスタを更新
            priv_data.regs[reg_idx] = (priv_data.regs[reg_idx] & 0xd0) | (val[i] & 0x2f);

            // 0x14 と 0x15 の 2バイトを書き込み
            self.write_regs(0x14, &priv_data.regs[0x14..0x16])?;

            // ADC値を読み取り
            let tmp = self.read_adc_value()?;

            // 最小値（最適な設定）を保持
            if imr_tmp.value > tmp {
                match direction {
                    R850ImrDirection::Gain => imr_tmp.gain = val[i],
                    R850ImrDirection::Phase => imr_tmp.phase = val[i],
                }
                imr_tmp.value = tmp;
            }
        }

        Ok(imr_tmp)
    }

    // IMRのゲインまたはフェーズのステップ探索（値を増やしながら追い込む）
    // ステップ状での探索アルゴリズム、らしい。
    fn imr_check_iq_step(
        &self,
        imr: R850Imr,
        direction: R850ImrDirection,
        priv_data: &mut R850Priv,
    ) -> Result<R850Imr, TunerError> {
        let reg_idx: usize;
        let mut val: u8;
        let mut imr_tmp = imr; // 現在の値をベースラインとして保持

        // 探索方向に応じたレジスタと初期値の設定
        match direction {
            R850ImrDirection::Gain => {
                reg_idx = 0x14; // ゲインレジスタ
                val = imr.gain;
                // 他方のレジスタ (Phase) を現在の値で固定
                priv_data.regs[0x15] = (priv_data.regs[0x15] & 0xd0) | (imr.phase & 0x2f);
            }
            R850ImrDirection::Phase => {
                reg_idx = 0x15; // フェーズレジスタ
                val = imr.phase;
                // 他方のレジスタ (Gain) を現在の値で固定
                priv_data.regs[0x14] = (priv_data.regs[0x14] & 0xd0) | (imr.gain & 0x2f);
            }
        }

        // 下位4ビットが8以下の間、値を増やしながら探索を続ける
        while (val & 0x0f) <= 8 {
            val = val.wrapping_add(1);

            // ターゲットレジスタを更新 (マスク 0x2f)
            priv_data.regs[reg_idx] = (priv_data.regs[reg_idx] & 0xd0) | (val & 0x2f);

            // 0x14, 0x15 の 2バイトをセットで書き込み
            self.write_regs(0x14, &priv_data.regs[0x14..0x16])?;

            // ADC値を読み取り
            let tmp = self.read_adc_value()?;

            if imr_tmp.value > tmp {
                // より良い値が見つかれば更新
                match direction {
                    R850ImrDirection::Gain => imr_tmp.gain = val,
                    R850ImrDirection::Phase => imr_tmp.phase = val,
                }
                imr_tmp.value = tmp;
            } else if (imr_tmp.value as u16 + 2) < tmp as u16 {
                // 現在のベストより明らかに誤差が増えた（+2より大きい）場合は
                // これ以上追いかけても改善しないと判断してループを抜ける
                break;
            }
        }

        Ok(imr_tmp)
    }

    // IMRのセクション探索（ゲインの周辺3点に対してフェーズ探索を行う）
    // 特定セクションの周辺探索、らしい。
    fn imr_check_section(
        &self,
        imr: R850Imr,
        priv_data: &mut R850Priv,
    ) -> Result<R850Imr, TunerError> {
        // 現在の状態をベースに 3つの候補点を作成
        // iqcap や phase は元の imr から引き継がれる
        let mut imr_points = [imr; 3];

        // ゲインの周辺3点を設定
        if imr.gain != 0 {
            imr_points[0].gain = imr.gain.wrapping_sub(1);
            // imr_points[1].gain は imr.gain そのまま
            imr_points[2].gain = imr.gain.wrapping_add(1);
        } else {
            // ゲインが 0 の場合の特殊なビット操作 (Cコードを忠実に再現)
            imr_points[0].gain = (imr.gain & 0xdf).wrapping_add(1);
            imr_points[2].gain = (imr.gain | 0x20).wrapping_add(1);
        }

        let mut best_idx = 0;
        let mut min_val = 0xffu8;

        for i in 0..3 {
            // Cコードでは imr_points[3] を宣言するだけなので、一旦、初期値にしておく。
            // imr_check_iq_tree() 内部で、.value を 0xff に初期化してから使うので不要
            //imr_points[i].value = 0;
            // 各ポイントについて、フェーズ方向の 3点探索を実行
            imr_points[i] =
                self.imr_check_iq_tree(imr_points[i], R850ImrDirection::Phase, 3, priv_data)?;

            // 最も ADC値 (value) が低い（＝誤差が少ない）インデックスを記録
            if min_val > imr_points[i].value {
                min_val = imr_points[i].value;
                best_idx = i;
            }
        }

        // 最良の結果となった IMR 設定を返す
        Ok(imr_points[best_idx])
    }

    // IMRのIQキャパシタ設定をチェックする
    fn imr_check_iqcap(
        &self,
        imr: R850Imr,
        priv_data: &mut R850Priv,
    ) -> Result<R850Imr, TunerError> {
        let mut imr_tmp = imr;

        // レジスタ 0x14 (Gain) の準備と書き込み
        priv_data.regs[0x14] = (priv_data.regs[0x14] & 0xd0) | (imr.gain & 0x2f);
        self.write_regs(0x14, &priv_data.regs[0x14..0x15])?;

        // レジスタ 0x15 (Phase) の準備（下位ビットをセット）
        priv_data.regs[0x15] = (priv_data.regs[0x15] & 0xd0) | (imr.phase & 0x2f);

        // 最小値を探すための初期化
        imr_tmp.iqcap = 0;
        imr_tmp.value = 0xff;

        // iqcap の 3つの候補 (0, 1, 2) を試す
        for i in 0..3u8 {
            // 0x15 の上位 2ビット (0xc0) を更新
            priv_data.regs[0x15] = (priv_data.regs[0x15] & 0x3f) | ((i << 6) & 0xc0);

            // レジスタ 0x15 のみを書き込み
            self.write_regs(0x15, &priv_data.regs[0x15..0x16])?;

            // ADC値を読み取り
            let tmp = self.read_adc_value()?;

            // 最小値を更新
            if tmp < imr_tmp.value {
                imr_tmp.iqcap = i;
                imr_tmp.value = tmp;
            }
        }

        Ok(imr_tmp)
    }

    // キャリブレーションの準備（レジスタマップをキャリブレーション用に書き換える）
    fn prepare_calibration(
        &self,
        cal: R850Calibration,
        priv_data: &mut R850Priv,
    ) -> Result<(), TunerError> {
        match cal {
            R850Calibration::IMR => {
                // IMRキャリブレーション用レジスタ配列で全上書き
                priv_data.regs.copy_from_slice(&IMR_CAL_REGS);
            }
            R850Calibration::LPF => {
                // LPFキャリブレーション用レジスタ配列で全上書き
                priv_data.regs.copy_from_slice(&LPF_CAL_REGS);
            }
            _ => return Err(TunerError::InvalidArgument),
        }

        // 移植元 Cコードで #if 0 になっているため、現在は何もしない。
        // もし将来的に必要になった場合は、ここで self.write_regs を呼ぶ。
        /*
        self.write_regs(0x08, &priv_data.regs[0x08..R850_NUM_REGS])?;
        */

        Ok(())
    }

    // IMRキャリブレーションの実行
    // 5つの異なるテスト周波数に対してダミーのPLL/MUX設定を行い、上記 imr_check_xxx 関数群を駆使してIMRのキャリブレーションをフルに実行し、結果を内部に保存する。
    fn calibrate_imr(&self, priv_data: &mut R850Priv) -> Result<(), TunerError> {
        //let mut priv_data = self.priv_.lock().unwrap();

        let n = [2, 1, 0, 3, 4];
        let mixer_mode = priv_data.mixer_mode;
        let mixer_amp_lpf = priv_data.mixer_amp_lpf_imr_cal;
        let mixer_mode_idx = mixer_mode as usize;

        for &j in &n {
            let ring_freq: u32;
            let mut full = false;
            let mut pre = 2;
            let j_idx = j as usize;

            // 周波数点に応じた設定
            match j {
                0 => {
                    ring_freq = 136000;
                    priv_data.regs[0x24] = (priv_data.regs[0x24] & 0xf0) | 0x0a;
                    pre = 1;
                }
                1 => {
                    ring_freq = 326400;
                    priv_data.regs[0x24] = (priv_data.regs[0x24] & 0xf0) | 0x05;
                }
                2 => {
                    ring_freq = 544000;
                    priv_data.regs[0x24] = (priv_data.regs[0x24] & 0xf0) | 0x02;
                    full = true;
                }
                3 => {
                    ring_freq = 816000;
                    priv_data.regs[0x24] &= 0xf0;
                    if mixer_mode != 0 {
                        full = true;
                    }
                }
                4 => {
                    ring_freq = 204000;
                    priv_data.regs[0x24] = (priv_data.regs[0x24] & 0xf0) | 0x08;
                    pre = 1;
                }
                _ => return Err(TunerError::InvalidState),
            }

            //ålet imr_tmp = priv_data.imr_cal[mixer_mode_idx].imr[j];

            priv_data.regs[0x23] = (priv_data.regs[0x23] & 0xa0) | 0x11;

            if mixer_mode == 0 {
                // Mixer Mode 0 の設定
                self.set_mux(
                    ring_freq - 5300,
                    ring_freq,
                    R850System::Undefined,
                    priv_data,
                );

                self.set_pll(ring_freq - 5300, 5300, R850System::Undefined, priv_data)?;

                priv_data.regs[0x13] = (priv_data.regs[0x13] & 0xe8) | (mixer_amp_lpf & 0x07);
                self.write_regs(0x13, &priv_data.regs[0x13..0x14])?;

                if j == 4 {
                    priv_data.regs[0x24] = (priv_data.regs[0x24] & 0xcf) | 0x10;
                } else {
                    priv_data.regs[0x24] |= 0x30;
                }
                self.write_regs(0x24, &priv_data.regs[0x24..0x25])?;

                priv_data.regs[0x29] = (priv_data.regs[0x29] & 0xf0) | 0x08;
                self.write_regs(0x29, &priv_data.regs[0x29..0x2a])?;
            } else {
                // Mixer Mode 1 の設定
                self.set_mux(
                    ring_freq + 5300,
                    ring_freq,
                    R850System::Undefined,
                    priv_data,
                );
                self.set_pll(ring_freq + 5300, 5300, R850System::Undefined, priv_data)?;

                priv_data.regs[0x13] =
                    (priv_data.regs[0x13] | 0x10) & 0xf8 | (mixer_amp_lpf & 0x07);
                self.write_regs(0x13, &priv_data.regs[0x13..0x14])?;

                priv_data.regs[0x29] &= 0xf0;
                if j == 4 {
                    priv_data.regs[0x29] |= 0x07;
                    priv_data.regs[0x24] = (priv_data.regs[0x24] & 0xcf) | 0x10;
                } else {
                    priv_data.regs[0x29] |= 0x06;
                    priv_data.regs[0x24] |= 0x30;
                }
                self.write_regs(0x29, &priv_data.regs[0x29..0x2a])?;
                self.write_regs(0x24, &priv_data.regs[0x24..0x25])?;
            }

            priv_data.regs[0x29] |= 0xf0;
            self.write_regs(0x29, &priv_data.regs[0x29..0x2a])?;

            // 探索処理の開始
            let mut current_imr: R850Imr;

            if full {
                let (imr_after_cross, dir) = self.imr_check_iq_cross(priv_data)?;
                current_imr = imr_after_cross;

                current_imr = self.imr_check_iq_step(current_imr, dir, priv_data)?;

                let opposite_dir = if dir == R850ImrDirection::Gain {
                    R850ImrDirection::Phase
                } else {
                    R850ImrDirection::Gain
                };

                current_imr = self.imr_check_iq_tree(current_imr, opposite_dir, 5, priv_data)?;
                current_imr = self.imr_check_iq_tree(current_imr, dir, 3, priv_data)?;
            } else {
                // 以前のポイントの結果を引き継ぐ
                current_imr = priv_data.imr_cal[mixer_mode_idx].imr[pre as usize];
            }

            // 共通の仕上げ探索
            current_imr = self.imr_check_section(current_imr, priv_data)?;
            current_imr = self.imr_check_iqcap(current_imr, priv_data)?;

            // 結果の保存 (Cコードでは、直接書き換えているが、Rust では異なるため)
            priv_data.imr_cal[mixer_mode_idx].imr[j_idx] = current_imr;

            if (current_imr.gain & 0x0f) <= 0x06 && (current_imr.phase & 0x0f) <= 0x06 {
                priv_data.imr_cal[mixer_mode_idx].result[j_idx] = true;
            } else {
                priv_data.imr_cal[mixer_mode_idx].result[j_idx] = false;
            }

            if full {
                // フル探索後はゲイン/フェーズ/iqcapのレジスタをリセット
                priv_data.regs[0x14] &= 0xd0;
                priv_data.regs[0x15] &= 0x10;
                self.write_regs(0x14, &priv_data.regs[0x14..0x16])?;
            }
        }

        priv_data.imr_cal[mixer_mode_idx].done = true;
        priv_data.imr_cal[mixer_mode_idx].mixer_amp_lpf = mixer_amp_lpf;

        Ok(())
    }

    // LPF（ローパスフィルタ）のキャリブレーション実行
    // ADCの値を監視しながら帯域幅（Bandwidth）とカットオフ周波数（Code / LSB）を微調整し、最適なLPFパラメータを決定する。
    fn calibrate_lpf(
        &self,
        if_freq: u32,
        bw: u8,
        gap: u8,
        priv_data: &mut R850Priv,
    ) -> Result<LPFparams, TunerError> {
        let mut val: u8 = 0;
        let mut val2: u8;
        let mut val3: u8 = 0;
        let mut bandwidth: u8 = 0;

        //let mut priv_data = self.priv_.lock().unwrap();

        // 初期 PLL 設定
        self.set_pll(72000 - if_freq, if_freq, R850System::Undefined, priv_data)?;

        // 1. レジスタ 0x29 の調整（AGC/ADCの基準出し）
        for i in 5..16u8 {
            priv_data.regs[0x29] = (priv_data.regs[0x29] & 0x0f) | ((i << 4) & 0xf0);
            self.write_regs(0x29, &priv_data.regs[0x29..0x2a])?;

            std::thread::sleep(std::time::Duration::from_millis(5));

            val = self.read_adc_value()?;
            if val > 0x28 {
                break;
            }
        }

        // 2. 高い中間周波数(IF)の場合の検証
        if if_freq > 9999 {
            self.set_pll(63500, 8500, R850System::Undefined, priv_data)?;
            std::thread::sleep(std::time::Duration::from_millis(5));
            val3 = self.read_adc_value()?;

            if val3 <= (val.wrapping_add(8)) {
                // 成功なら元のPLL設定に戻す
                self.set_pll(72000 - if_freq, if_freq, R850System::Undefined, priv_data)?;
            } else {
                return Err(TunerError::CalibrationFailed); // キャリブレーション失敗
            }
        }

        // 3. 帯域幅 (Bandwidth) の探索
        let start_i = if bw == 2 { 1 } else { 0 };
        for i in start_i..3 {
            bandwidth = if i == 0 { 0 } else { i + 1 };

            // レジスタ 0x17 の更新（帯域幅設定）
            priv_data.regs[0x17] &= 0x9f;
            priv_data.regs[0x17] &= 0xe1;
            priv_data.regs[0x17] |= (bandwidth << 5) & 0x60;
            self.write_regs(0x17, &priv_data.regs[0x17..0x18])?;

            std::thread::sleep(std::time::Duration::from_millis(5));
            val = self.read_adc_value()?;

            // 比較用の一時的な設定
            priv_data.regs[0x17] = (priv_data.regs[0x17] & 0xe1) | 0x1a;
            self.write_regs(0x17, &priv_data.regs[0x17..0x18])?;

            std::thread::sleep(std::time::Duration::from_millis(5));
            val2 = self.read_adc_value()?;

            if (val2 as u16 + 16) < val as u16 {
                break;
            }
        }

        // 4. LPF Code (遮断周波数コード) の探索
        let mut lpf = LPFparams {
            bandwidth,
            code: 0,
            lsb: 0,
        };

        let mut final_i: u8 = 16; // デフォルト値（見つからなかった場合）
        let mut baseline_val = val;

        for i in 0..16u8 {
            priv_data.regs[0x17] = (priv_data.regs[0x17] & 0xe1) | ((i << 1) & 0x1e);
            self.write_regs(0x17, &priv_data.regs[0x17..0x18])?;

            std::thread::sleep(std::time::Duration::from_millis(5));
            val2 = self.read_adc_value()?;

            if i == 0 {
                baseline_val = if if_freq <= 9999 { val2 } else { val3 };
            }

            if (val2 as u16 + gap as u16) < baseline_val as u16 {
                if i == 0 {
                    return Err(TunerError::CalibrationFailed);
                }

                // 微調整 (LSBビットの確認)
                priv_data.regs[0x17] =
                    (priv_data.regs[0x17] & 0xe0) | 0x01 | (((i - 1) << 1) & 0x1e);
                self.write_regs(0x17, &priv_data.regs[0x17..0x18])?;

                std::thread::sleep(std::time::Duration::from_millis(5));
                val2 = self.read_adc_value()?;

                let mut adjusted_i = i;
                if (val2 as u16 + gap as u16) < baseline_val as u16 {
                    adjusted_i = i - 1;
                    lpf.lsb = 1;
                }

                final_i = adjusted_i;
                break;
            }
            final_i = i; // ループが最後まで回った場合用
        }

        lpf.code = final_i;
        Ok(lpf)
    }

    // 放送方式（システム）に応じたパラメータの設定
    // 選局時に呼ばれる処理で、必要に応じて IMR および LPF のキャリブレーション（calibrate_imr, calibrate_lpf）をトリガーし、SYS_PARAMS テーブルから取得したシステム固有の定数（Notch, ゲイン設定など）をレジスタに適用する。
    pub fn set_system_params(&self, priv_data: &mut R850Priv) -> Result<(), TunerError> {
        // システムが未定義の場合はエラー
        if priv_data.sys.system == R850System::Undefined {
            return Err(TunerError::InvalidArgument);
        }

        // 1. IMRキャリブレーションが必要か判定
        let mixer_idx = priv_data.mixer_mode as usize;
        let needs_imr = !self.config.no_imr_calibration
            && (!priv_data.imr_cal[mixer_idx].done
                || priv_data.imr_cal[mixer_idx].mixer_amp_lpf != priv_data.mixer_amp_lpf_imr_cal);

        if needs_imr {
            // キャリブレーション準備
            self.prepare_calibration(R850Calibration::IMR, priv_data)?;
            self.calibrate_imr(priv_data)?;
        }

        // 2. システム設定に変更があるかチェック
        // R850SystemConfig が PartialEq を実装していることを前提としています
        if priv_data.sys != priv_data.sys_curr {
            let sys = priv_data.sys;
            let chip_idx = priv_data.chip as usize;
            let sys_idx = sys.system as usize;

            // パラメータテーブルから該当する設定を検索
            let prm = SYS_PARAMS[sys_idx][chip_idx]
                .iter()
                .find(|p| p.bandwidth == sys.bandwidth && p.if_freq == sys.if_freq)
                .ok_or(TunerError::InvalidArgument)?;

            // 3. LPFキャリブレーション
            let lpf_params = if !self.config.no_lpf_calibration {
                self.prepare_calibration(R850Calibration::LPF, priv_data)?;
                self.calibrate_lpf(prm.filt_cal_if, prm.bw, 2, priv_data)?
            } else {
                prm.lpf
            };

            // レジスタを初期状態に戻す
            self.init_regs(priv_data);

            // 4. レジスタ値の更新
            // 0x17: [7] HPF Notch, [6:5] BW, [4:1] Code, [0] LSB
            priv_data.regs[0x17] = (lpf_params.lsb & 0x01)
                | ((lpf_params.code << 1) & 0x1e)
                | ((lpf_params.bandwidth << 5) & 0x60)
                | ((prm.hpf_notch << 7) & 0x80);

            // 0x18: [7:4] HPF Cor, [3:2] Filt Comp
            priv_data.regs[0x18] = (priv_data.regs[0x18] & 0x0f) | ((prm.hpf_cor << 4) & 0xf0);
            priv_data.regs[0x18] = (priv_data.regs[0x18] & 0xf3) | ((prm.filt_comp << 2) & 0x0c);

            // 0x12: [6] Filt Ext Ena
            priv_data.regs[0x12] = (priv_data.regs[0x12] & 0xbf) | ((prm.filt_ext_ena << 6) & 0x40);

            // 0x2f: [3:2] AGC Clk
            priv_data.regs[0x2f] = (priv_data.regs[0x2f] & 0xf3) | ((prm.agc_clk << 2) & 0x0c);

            // チップタイプに応じた Image Gain の設定 (0x2c, 0x2e)
            if priv_data.chip != 0 {
                priv_data.regs[0x2c] = (priv_data.regs[0x2c] & 0xfe) | ((prm.img_gain >> 1) & 0x01);
            }
            priv_data.regs[0x2e] = (priv_data.regs[0x2e] & 0xef) | ((prm.img_gain << 4) & 0x10);

            // 移植元 Cコードの #if 0 部分（必要に応じて self.write_regs を呼ぶ）
            /*
            self.write_regs(0x08, &priv_data.regs[0x08..R850_NUM_REGS])?;
            */

            // 現在の設定を保存
            priv_data.sys_curr = priv_data.sys;
        }

        Ok(())
    }

    // 選局時に呼ばれる処理で、要求された周波数（rf_freq）に合致するパラメータを SYS_FREQ_PARAMS（IsdbT_FREQ_PARAMS など）から探し出し、LNA（低雑音増幅器）やミキサーのゲイン・電圧設定を反映させた上で、最終的に set_mux と set_pll を呼び出して周波数をロックする。
    fn set_system_frequency(
        &self,
        rf_freq: u32,
        priv_data: &mut R850Priv,
    ) -> Result<(), TunerError> {
        // 1. システムに対応するパラメータテーブルを取得
        let params_table: &[R850SystemFrequencyParams] = match priv_data.sys_curr.system {
            R850System::DvbT | R850System::DvbT2 | R850System::DvbT2_1 => &DVB_T_T2_FREQ_PARAMS,
            R850System::DvbC => &DVB_C_FREQ_PARAMS,
            R850System::J83B => &J83B_FREQ_PARAMS,
            R850System::IsdbT => &ISDB_T_FREQ_PARAMS,
            R850System::Dtmb => &DTMB_FREQ_PARAMS,
            R850System::Atsc => &ATSC_FREQ_PARAMS,
            _ => return Err(TunerError::InvalidState), // 定義されていないシステム
        };

        // 2. 条件に合うパラメータを検索
        let mut prm = None;
        for p in params_table {
            if (p.if_freq == 0 || p.if_freq == priv_data.sys_curr.if_freq)
                && (p.rf_freq_min == 0 || p.rf_freq_min <= rf_freq)
                && (p.rf_freq_max == 0 || p.rf_freq_max >= rf_freq)
            {
                prm = Some(*p);
                break;
            }
        }

        let mut prm = prm.ok_or(TunerError::InvalidState)?;

        // 3. チップの種類に応じた微調整
        match priv_data.sys_curr.system {
            R850System::DvbC | R850System::J83B | R850System::IsdbT => {
                if priv_data.chip != 0 {
                    prm.filter_top = 6;
                }
            }
            _ => {}
        }

        // 4. レジスタの更新処理
        // Mixer Mode と LO周波数の計算
        priv_data.regs[0x13] &= 0xef;
        let lo_freq: u32;
        if priv_data.mixer_mode != 0 {
            priv_data.regs[0x13] |= 0x10;
            lo_freq = rf_freq - priv_data.sys_curr.if_freq;
        } else {
            lo_freq = rf_freq + priv_data.sys_curr.if_freq;
        }

        // NA Power Detect
        priv_data.regs[0x0a] &= 0xbf;
        priv_data.regs[0x0a] |= (prm.na_pwr_det << 6) & 0x40;

        // 初期設定レジスタからの反映
        priv_data.regs[0x10] &= 0xdf;
        priv_data.regs[0x10] |= INIT_REGS[0x0c] & 0x20;

        // LNA NRB Detect
        priv_data.regs[0x0b] &= 0x7f;
        priv_data.regs[0x0b] |= (prm.lna_nrb_det << 7) & 0x80;

        // LNA Top
        priv_data.regs[0x26] &= 0xf8;
        priv_data.regs[0x26] |= (7 - prm.lna_top) & 0x07;

        priv_data.regs[0x27] = prm.lna_vtl_h;

        // RF LTE PSG
        priv_data.regs[0x11] &= 0xef;
        priv_data.regs[0x11] |= (prm.rf_lte_psg << 4) & 0x10;

        // RF Top
        priv_data.regs[0x26] &= 0x8f;
        priv_data.regs[0x26] |= ((7 - prm.rf_top) << 4) & 0x70;

        priv_data.regs[0x2a] = prm.rf_vtl_h;

        // RF Gain Limit
        if prm.rf_gain_limit <= 3 {
            if prm.rf_gain_limit < 2 {
                priv_data.regs[0x12] &= 0xfb;
            } else {
                priv_data.regs[0x12] |= 0x02;
            }

            if (prm.rf_gain_limit % 2) != 0 {
                priv_data.regs[0x10] |= 0x40;
            } else {
                priv_data.regs[0x10] &= 0xbf;
            }
        }

        // Mixer Amp LPF
        priv_data.regs[0x13] &= 0xf8;
        priv_data.regs[0x13] |= prm.mixer_amp_lpf & 0x07;

        // Mixer Top
        priv_data.regs[0x28] &= 0xf0;
        priv_data.regs[0x28] |= (15 - prm.mixer_top) & 0x0f;

        // Filter Top
        if priv_data.chip != 0 {
            priv_data.regs[0x2c] &= 0xf1;
            priv_data.regs[0x2c] |= ((7 - prm.filter_top) << 1) & 0x0e;
        } else {
            priv_data.regs[0x2c] &= 0xf0;
            priv_data.regs[0x2c] |= (15 - prm.filter_top) & 0x0f;
        }

        // Filt 3th LPF Current / Gain
        priv_data.regs[0x0a] &= 0xef;
        priv_data.regs[0x0a] |= (prm.filt_3th_lpf_cur << 4) & 0x10;

        priv_data.regs[0x18] &= 0xfc;
        priv_data.regs[0x18] |= prm.filt_3th_lpf_gain & 0x03;

        // VTH / VTL
        priv_data.regs[0x29] = ((prm.filter_vth << 4) & 0xf0) | (prm.mixer_vth & 0x0f);
        priv_data.regs[0x2b] = ((prm.filter_vtl << 4) & 0xf0) | (prm.mixer_vtl & 0x0f);

        // Mixer Gain Limit
        priv_data.regs[0x16] &= 0x3f;
        priv_data.regs[0x16] |= (prm.mixer_gain_limit << 6) & 0xc0;

        // Mixer DetBW LPF
        priv_data.regs[0x2e] &= 0x7f;
        priv_data.regs[0x2e] |= (prm.mixer_detbw_lpf << 7) & 0x80;

        // LNA RF DIS Mode
        match prm.lna_rf_dis_mode {
            1 => {
                priv_data.regs[0x2d] |= 0x03;
                priv_data.regs[0x1f] |= 0x01;
                priv_data.regs[0x20] |= 0x20;
            }
            2 => {
                priv_data.regs[0x2d] |= 0x03;
                priv_data.regs[0x1f] &= 0xfe;
                priv_data.regs[0x20] &= 0xdf;
            }
            3 => {
                priv_data.regs[0x2d] |= 0x03;
                priv_data.regs[0x1f] |= 0x01;
                priv_data.regs[0x20] &= 0xdf;
            }
            4 => {
                priv_data.regs[0x2d] |= 0x03;
                priv_data.regs[0x1f] &= 0xfe;
                priv_data.regs[0x20] |= 0x20;
            }
            _ => {
                priv_data.regs[0x2d] &= 0xfc;
                priv_data.regs[0x1f] |= 0x01;
                priv_data.regs[0x20] |= 0x20;
            }
        }

        // その他 LNA/RF/BB 関連設定
        priv_data.regs[0x1f] &= 0xfd;
        priv_data.regs[0x1f] |= (prm.lna_rf_charge_cur << 1) & 0x02;

        priv_data.regs[0x0d] &= 0xdf;
        priv_data.regs[0x0d] |= (prm.lna_rf_dis_curr << 5) & 0x20;

        priv_data.regs[0x2d] &= 0x0f;
        priv_data.regs[0x2d] |= (prm.rf_dis_slow_fast << 4) & 0xf0;

        priv_data.regs[0x2c] &= 0x0f;
        priv_data.regs[0x2c] |= (prm.lna_dis_slow_fast << 4) & 0xf0;

        priv_data.regs[0x19] &= 0xbf;
        priv_data.regs[0x19] |= (prm.bb_dis_curr << 6) & 0x40;

        priv_data.regs[0x25] &= 0x3b;
        priv_data.regs[0x25] |=
            ((prm.mixer_filter_dis << 6) & 0xc0) | ((prm.bb_det_mode << 2) & 0x04);

        priv_data.regs[0x19] &= 0xfd;
        priv_data.regs[0x19] |= (prm.enb_poly_gain << 1) & 0x02;

        // NRB Top
        priv_data.regs[0x28] &= 0x0f;
        priv_data.regs[0x28] |= ((15 - prm.nrb_top) << 4) & 0xf0;

        // NRB BW LPF/HPF
        priv_data.regs[0x1a] &= 0x33;
        priv_data.regs[0x1a] |= ((prm.nrb_bw_lpf << 6) & 0xc0) | ((prm.nrb_bw_hpf << 2) & 0x0c);

        // Image NRB Adder
        priv_data.regs[0x2e] &= 0xf3;
        priv_data.regs[0x2e] |= (prm.img_nrb_adder << 2) & 0x0c;

        // HPF Comp
        priv_data.regs[0x0d] &= 0xf9;
        priv_data.regs[0x0d] |= (prm.hpf_comp << 1) & 0x06;

        // FB Res 1st
        priv_data.regs[0x15] &= 0xef;
        priv_data.regs[0x15] |= (prm.fb_res_1st << 4) & 0x10;

        // ISDB-T 固有の微調整
        if priv_data.sys_curr.system == R850System::IsdbT && (478000..=481999).contains(&rf_freq) {
            // Cコードの (rf_freq - 478000) <= 3999 は 474001~481999 の範囲
            priv_data.regs[0x2f] &= 0xf3;
        }

        priv_data.regs[0x19] &= 0xdf;

        // Loop Through 設定
        if self.config.loop_through {
            priv_data.regs[0x08] |= 0xc0;
            priv_data.regs[0x0a] |= 0x02;
        } else {
            priv_data.regs[0x08] &= 0x3f;
            priv_data.regs[0x08] |= 0x40;
            priv_data.regs[0x0a] &= 0xfd;
        }

        // Clock Out 設定
        if self.config.clock_out {
            priv_data.regs[0x22] &= 0xfb;
        } else {
            priv_data.regs[0x22] |= 0x04;
        }

        // 5. 下位層の関数呼び出し (ロックを解除してから呼ぶ必要がある場合は調整が必要)
        let system = priv_data.sys_curr.system;
        let if_freq = priv_data.sys_curr.if_freq;

        self.set_mux(rf_freq, lo_freq, system, priv_data);
        self.set_pll(lo_freq, if_freq, system, priv_data)?;

        Ok(())
    }

    // 0〜3 の範囲で水晶発振器のパワー設定を変えながら PLL がロックするかどうかテストし、最適なパワー設定値を探し出す。
    pub fn check_xtal_power(&self, priv_data: &mut R850Priv) -> Result<(), CtrlMsgError> {
        // debug
        println!("[R850] call check_xtal_power().");

        let bank = 55u8;
        let mut pwr = 3u8; // xtal 24MHz

        // 保持するレジスタ状態を初期化し、xtal power 確認のために変更後のレジスタ状態を保持
        self.init_regs(priv_data);

        //let mut priv_data = self.priv_.lock().unwrap();

        if priv_data.chip != 0 {
            priv_data.regs[0x2f] &= 0xfd;
        } else {
            priv_data.regs[0x2f] &= 0xfc;
        }

        priv_data.regs[0x1b] &= 0x80;
        priv_data.regs[0x1b] |= 0x12;

        priv_data.regs[0x1e] &= 0xe0;
        priv_data.regs[0x1e] |= 0x08;

        priv_data.regs[0x22] &= 0x27;

        priv_data.regs[0x1d] &= 0x0f;

        priv_data.regs[0x21] |= 0xf8;

        priv_data.regs[0x22] &= 0x77;
        priv_data.regs[0x22] |= 0x80;

        priv_data.regs[0x1f] &= 0x80;
        priv_data.regs[0x1f] |= 0x40;
        priv_data.regs[0x1f] &= 0xbf;

        // 本体のレジスタに書き込み
        self.write_regs(0x08, &priv_data.regs[0x08..R850_NUM_REGS])?;

        // ループで xtal_power を探す
        for i in 0..=3 {
            priv_data.regs[0x22] &= 0xcf;
            priv_data.regs[0x22] |= i << 4;

            self.write_regs(0x22, &[priv_data.regs[0x22]])?;

            let mut tmp = [0u8; 1];
            self.read_regs(0x02, &mut tmp)?;

            if (tmp[0] & 0x40) != 0 && ((tmp[0] & 0x3f) as i32 - (bank as i32 - 6) <= 12) {
                pwr = i;
                break;
            }
        }

        if pwr < 3 {
            pwr += 1;
        }

        priv_data.xtal_pwr = pwr;

        Ok(())
    }

    // インスタンス生成
    pub fn new(
        it930x: &'a IT930x<B>,
        tc90522_bus: u8,
        tc90522_addr: u8,
        is_secondary: bool,
    ) -> Result<Self, TunerError> {
        // debug
        println!("[R850] new()");

        let tc90522 = TC90522::new(
            it930x,
            tc90522_bus,
            tc90522_addr,
            System::IsdbT,
            is_secondary,
        );

        // 生成された直後に、この論理コアをスリープ状態にする
        //tc90522.sleep(true)?;
        // まだ、ちゃんと立ち上がってなくて、送れないっぽい

        Ok(Self {
            tc90522,
            i2c_addr: 0x7c,

            config: R850Config {
                xtal: 24000,
                //loop_through: false,
                loop_through: !is_secondary, // チューナー生成時に、i==2 が true、i==3 が false で、is_secondary と逆なので。
                clock_out: false,
                no_imr_calibration: false,
                no_lpf_calibration: false,
            },

            priv_: Mutex::new(R850Priv {
                //lock: Mutex::new(()),
                init: false,
                chip: 0,
                xtal_pwr: 0,
                regs: [0u8; R850_NUM_REGS],
                sleep: false,
                sys: R850SystemConfig {
                    system: R850System::Undefined,
                    bandwidth: R850Bandwidth::B6M,
                    if_freq: 0,
                },
                sys_curr: R850SystemConfig {
                    system: R850System::Undefined,
                    bandwidth: R850Bandwidth::B6M,
                    if_freq: 0,
                },
                imr_cal: [
                    R850ImrCal {
                        imr: [R850Imr {
                            gain: 0,
                            phase: 0,
                            iqcap: 0,
                            value: 0,
                        }; 5],
                        done: false,
                        result: [false; 5],
                        mixer_amp_lpf: 0,
                    },
                    R850ImrCal {
                        imr: [R850Imr {
                            gain: 0,
                            phase: 0,
                            iqcap: 0,
                            value: 0,
                        }; 5],
                        done: false,
                        result: [false; 5],
                        mixer_amp_lpf: 0,
                    },
                ],
                mixer_mode: 0,
                mixer_amp_lpf_imr_cal: 0,
            }),
        })
    }

    // チューナーをスリープ状態に移行
    fn sleep(&self, priv_data: &mut R850Priv) -> Result<(), TunerError> {
        //let mut priv_data = self.priv_.lock().unwrap();

        if !priv_data.init {
            return Err(TunerError::InvalidState);
        }

        if priv_data.sleep {
            return Ok(());
        }

        priv_data.regs.copy_from_slice(&SLEEP_REGS);

        if !self.config.loop_through {
            priv_data.regs[0x08] |= 0x40;
        }

        // debug
        println!(
            "[debug] r850.sleep chip={:?} loop_through={} i2c_addr={:02X} r08=0x{:02x}",
            priv_data.chip, self.config.loop_through, self.i2c_addr, priv_data.regs[0x08]
        );

        self.write_regs(0x08, &priv_data.regs[0x08..R850_NUM_REGS])?;

        Ok(())
    }

    // チューナーを立ち上げ状態に移行
    pub fn wakeup(&self) -> Result<(), TunerError> {
        let mut priv_data = self.priv_.lock().unwrap();

        // 初期化されていない場合はエラー
        if !priv_data.init {
            return Err(TunerError::InvalidState);
        }

        // スリープ状態でなければ何もしない
        if !priv_data.sleep {
            return Ok(());
        }

        // 1. レジスタキャッシュを起動用のパラメータに設定
        priv_data.regs.copy_from_slice(&WAKEUP_REGS);

        // 2. レジスタの書き込み (0x08 以降の範囲)
        // デバイスをスリープから復帰させるためのレジスタ変更を反映させます
        self.write_regs(0x08, &priv_data.regs[0x08..])?;

        // 3. レジスタの初期設定（内部状態やチップ固有設定の再適用）
        // これにより、priv_data.regs の内容が現在の設定値で更新されます
        self.init_regs(&mut priv_data);

        // 4. 更新されたレジスタキャッシュを再度デバイスへ書き込む
        self.write_regs(0x08, &priv_data.regs[0x08..])?;

        // 5. スリープ解除フラグを更新
        priv_data.sleep = false;

        Ok(())
    }

    // 対象とする放送規格（例：ISDB-T）を設定する。
    // 規格に応じてミキサーの動作モードや、IMRキャリブレーション用のLPFアンプ設定を決定する。
    pub fn set_system(&self, system_config: R850SystemConfig) -> Result<(), TunerError> {
        let mut priv_data = self.priv_.lock().unwrap();

        // 1. 初期化チェック
        if !priv_data.init {
            return Err(TunerError::InvalidState);
        }

        // 2. 放送規格に応じた mixer_mode と mixer_amp_lpf_imr_cal の決定
        let (mixer_mode, mixer_amp_lpf_imr_cal) = match system_config.system {
            R850System::DvbT
            | R850System::DvbT2
            | R850System::DvbT2_1
            | R850System::DvbC
            | R850System::Fm => (1, 4),
            R850System::J83B | R850System::Dtmb | R850System::Atsc => (0, 7),
            R850System::IsdbT => (1, 7),
            _ => return Err(TunerError::InvalidState),
        };

        // 3. 内部状態の更新
        // Cコードの t->priv.sys = *system に相当
        priv_data.sys = system_config;
        priv_data.mixer_mode = mixer_mode;
        priv_data.mixer_amp_lpf_imr_cal = mixer_amp_lpf_imr_cal;

        // 4. 現在適用中のシステムを「未定義」にリセット
        // これにより、次回の周波数設定時に確実にパラメータが再計算されるようになります
        priv_data.sys_curr.system = R850System::Undefined;

        Ok(())
    }

    // 選局用のメイン関数
    // 周波数の範囲チェックを行った後、set_system_params と set_system_frequency を順に実行する。
    pub fn set_frequency(&self, freq: u32) -> Result<(), TunerError> {
        // 1. 初期化チェックと周波数範囲のバリデーション
        // ロックを取得する前に、基本的な引数チェックを済ませます
        let mut priv_data = self.priv_.lock().unwrap();

        if !priv_data.init {
            return Err(TunerError::InvalidState);
        }

        // 40MHz 〜 1002MHz の範囲外ならエラー
        if freq < 40000 || freq > 1002000 {
            return Err(TunerError::InvalidState); // CのEINVAL相当
        }

        // 2. システムパラメータの設定 (r850_set_system_params)
        // ※この関数は以前のやり取りで触れた「キャリブレーションや基本フィルタ設定」を行うものと想定
        self.set_system_params(&mut priv_data)?;

        // 3. 放送規格固有の周波数設定 (r850_set_system_frequency)
        self.set_system_frequency(freq, &mut priv_data)?;

        Ok(())
    }

    // 指定した周波数でPLLが正常にロックしたかを判定する。
    pub fn is_pll_locked(&self) -> Result<bool, TunerError> {
        let mut tmp = [0u8; 1];
        let priv_data = self.priv_.lock().unwrap();

        // 1. 初期化チェック
        // read_regs 内でもチェックされるかもしれませんが、
        if !priv_data.init {
            return Err(TunerError::InvalidState);
        }

        // 2. レジスタ 0x02 を読み込む
        // r850_read_regs(t, 0x02, &tmp, 1) に相当
        if let Err(e) = self.read_regs(0x02, &mut tmp) {
            // Cコードの dev_err() に相当するエラーログ出力
            eprintln!("r850_is_pll_locked: read_regs() failed. ({:?})", e);
            return Err(TunerError::CtrlMsg(e));
        }

        // 3. ロック状態の判定
        // レジスタ 0x02 の bit 6 (0x40) が 1 ならロック成功
        let locked = (tmp[0] & 0x40) != 0;

        Ok(locked)
    }
}

impl<'a, B: BusOps> Tuner for R850<'a, B> {
    // 初期化処理
    fn init(&mut self) -> Result<(), TunerError> {
        // debug
        println!("[RT710] init()");

        // 初期状態の設定
        let mut priv_data = self.priv_.lock().unwrap();

        priv_data.init = false;

        priv_data.chip = 0;
        priv_data.sleep = false;

        priv_data.sys.system = R850System::Undefined;

        priv_data.sys_curr.system = R850System::Undefined;

        // なんか、Cコードとは別に、他の箇所の初期値を設定している。
        for cal in priv_data.imr_cal.iter_mut() {
            cal.done = false;
            cal.result = [false; 5];
            cal.mixer_amp_lpf = 0;
            for imr in cal.imr.iter_mut() {
                *imr = R850Imr {
                    gain: 0,
                    phase: 0,
                    iqcap: 0,
                    value: 0,
                };
            }
        }

        // チップ判定
        let mut detected = false;
        for _ in 0..4 {
            let mut tmp = [0u8];
            if self.read_regs(0x00, &mut tmp).is_ok() {
                if (tmp[0] & 0x98) != 0 {
                    priv_data.chip = 1;
                    detected = true;
                    break;
                }
            }
        }

        if !detected {
            return Err(TunerError::ChipNotDetected);
        }

        // レジスタ初期化
        let mut regs = [0u8; R850_NUM_REGS - 0x08];
        self.read_regs(0x08, &mut regs)?;

        // check xtal power
        self.check_xtal_power(&mut priv_data)?;

        self.write_regs(0x08, &regs)?;

        // init regs
        self.init_regs(&mut priv_data);

        priv_data.init = true;

        // いらないのでは？
        println!(
            "R850 init done. chip: {:?}, reg08=0x{:02x}",
            priv_data.chip, regs[0]
        );

        Ok(())
    }

    // デバイスの利用を開始する
    // px4_device.c の一部の機能を切り出し
    fn open(&self) -> Result<(), TunerError> {
        // debug
        println!("[R850] call open().");

        // 1. 個別ウェイクアップレジスタ (tc_init_t) の書き込み
        self.tc90522.write_multiple_regs(&TC_INIT_T)?;

        // 2. TSピンの無効化
        self.tc90522.enable_ts_pins(false)?;

        // 3. 復調器のスリープ解除
        self.tc90522.sleep(false)?;

        // 4. R850 チューナーチップ自身のウェイクアップ (Cコードの r850_wakeup 相当)
        self.wakeup()?;

        // 5. 初期システム・帯域・IF周波数の設定 (Cコードの r850_set_system 相当)
        // Cコード: sys.system = R850_SYSTEM_IsdbT; sys.bandwidth = R850_BANDWIDTH_6M; sys.if_freq = 4063;
        let sytem_config = R850SystemConfig {
            system: R850System::IsdbT,
            bandwidth: R850Bandwidth::B6M,
            if_freq: 4063,
        };
        self.set_system(sytem_config)?;

        Ok(())
    }

    // デバイスの利用を終了する
    // px4_device.c の一部の機能を切り出し
    fn close(&self) -> Result<(), TunerError> {
        // debug
        println!("[R850] call close().");

        let mut priv_data = self.priv_.lock().unwrap();

        // 逆の順序で終了させる
        // 1. チューナー自身をスリープ
        self.sleep(&mut priv_data)?;

        // 2. 復調器の TS出力 を無効化
        self.tc90522.enable_ts_pins(false)?;

        // 3. 復調器をスリープ
        self.tc90522.sleep(true)?;

        println!("[R850] Device closed and put to sleep.");
        Ok(())
    }

    fn init_0(&self) -> Result<(), TunerError> {
        // px4_device.c のコードの一部を切り出して、R850の役割として貼り付け
        // 492行目の処理で、Tuner の オープン1個目のときに走らせる。
        println!("[R850] Performing global demodulator initialization (T0)...");
        self.tc90522.write_multiple_regs(&TC_INIT_T0)?;
        Ok(())
    }

    fn tune(&mut self, freq: u32) -> Result<(), TunerError> {
        // debug
        println!("[R850] tune(): freq = {}", freq);

        // px4_device.c のコードの一部を切り出して、R850の役割として貼り付け
        // px4_chrdev_tune_t() の移植
        // 1. AGC設定
        self.tc90522.write_regs(0x47, &[0x30])?;
        self.tc90522.set_agc(false)?;
        self.tc90522.write_regs(0x76, &[0x0c])?;

        // 2. 周波数設定
        self.set_frequency(freq)?;

        // 3. PLLロック待ち (50回 * 10ms = 500ms)
        let mut locked = false;
        for _ in 0..50 {
            if self.is_pll_locked()? {
                locked = true;
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }

        if !locked {
            return Err(TunerError::InvalidState); // EAGAIN相当
        }

        // 4. AGC復帰と事後レジスタ設定
        self.tc90522.set_agc(true)?;
        self.tc90522
            .write_multiple_regs(&[(0x71, &[0x21]), (0x72, &[0x25]), (0x75, &[0x08])])?;

        Ok(())
    }

    fn is_locked(&self) -> Result<bool, TunerError> {
        // px4_device.c のコードの一部を切り出して、R850の役割として貼り付け
        // px4_chrdev_check_lock_t() の移植
        // tc90522.rs の `is_signal_locked` を呼び出す (CtrlMsgErrorは ? で自動変換されます)
        let locked = self.tc90522.is_signal_locked()?;
        Ok(locked)
    }

    fn enable_ts_pins(&mut self, enable: bool) -> Result<(), TunerError> {
        // 内部の tc90522 に対して enable_ts_pins を呼ぶ
        self.tc90522.enable_ts_pins(enable)?;
        Ok(())
    }

    fn read_cnr_raw(&self) -> Result<u32, TunerError> {
        self.tc90522.get_cn().map_err(|e| TunerError::CtrlMsg(e))
    }

    /// 明示的な終了処理
    fn term(&mut self) -> Result<(), TunerError> {
        println!("[info] R850 terminating tuner...");
        {
            // Mutexをロックして内部データにアクセスします。
            // 既に他のスレッドでパニックが発生してポイズニングされている可能性を考慮し、
            // if let で安全にロックを取得します。
            if let Ok(mut priv_data) = self.priv_.lock() {
                // 初期化されていない場合はクリーンアップ不要
                if !priv_data.init {
                    return Ok(());
                }

                let _ = self.sleep(&mut priv_data);

                // 1. システム状態を未定義に戻す (R850_SYSTEM_UNDEFINED 相当)
                priv_data.sys.system = R850System::Undefined;
                priv_data.sys_curr.system = R850System::Undefined;

                // 2. IMRキャリブレーション完了フラグのリセット
                priv_data.imr_cal[0].done = false;
                priv_data.imr_cal[1].done = false;

                // 3. レジスタキャッシュのゼロクリア (memset 相当)
                priv_data.regs.fill(0);

                // 4. チップ情報のクリア
                priv_data.chip = 0;

                // 5. 初期化フラグを倒す
                priv_data.init = false;
            }
        }
        // 3. 内包する復調器 (TC90522) の終了処理を連鎖させる
        self.tc90522.term()?;

        Ok(())
    }
}

impl<'a, B: BusOps> Drop for R850<'a, B> {
    // インスタンス破棄時に、内部状態をクリア
    fn drop(&mut self) {
        // Mutexをロックして内部データにアクセスします。
        // 既に他のスレッドでパニックが発生してポイズニングされている可能性を考慮し、
        // if let で安全にロックを取得します。
        if let Ok(mut priv_data) = self.priv_.lock() {
            // 初期化されていない場合はクリーンアップ不要
            if !priv_data.init {
                return;
            }

            let _ = self.sleep(&mut priv_data);

            // 1. システム状態を未定義に戻す (R850_SYSTEM_UNDEFINED 相当)
            priv_data.sys.system = R850System::Undefined;
            priv_data.sys_curr.system = R850System::Undefined;

            // 2. IMRキャリブレーション完了フラグのリセット
            priv_data.imr_cal[0].done = false;
            priv_data.imr_cal[1].done = false;

            // 3. レジスタキャッシュのゼロクリア (memset 相当)
            priv_data.regs.fill(0);

            // 4. チップ情報のクリア
            priv_data.chip = 0;

            // 5. 初期化フラグを倒す
            priv_data.init = false;
        }
    }
}
