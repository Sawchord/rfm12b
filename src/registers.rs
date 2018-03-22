
use core::marker::PhantomData;
use register::OffsetSize;
use register::{Mask, Read, Write};
// Holds the register addresses of the 17 control registers


#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum Register{
    ConfigSetting = 0x80,
    PowerManagement = 0x82,
    FreqSetting = 0xA0,
    DataRate = 0xC6,
    RxControl = 0x90,
    DataFilter = 0xC2,
    FifoAndResetMode = 0xCA,
    SynchPattern = 0xCE,
    ReceiverFifo = 0xB0,
    Afc = 0xC4,
    TxConfig = 0x98,
    PllSetting = 0xCC,
    TxRegisterWrite = 0xB8,
    WakeUpTimer = 0xE0,
    LowDutyCycle = 0xC8,
    LowBatDetectorAndCdiv = 0xC0,
    StatusRead = 0x00,
}

impl Register {
    pub(crate) fn addr(&self) -> u8 {
        *self as u8
    }
}


// TODO: Figure out, why this does not work
//impl Into<::Register> for Register {
//    fn into(self) -> ::Register {
//        ::Register::Common(self)
//    }
//}

register!(ConfigSetting, 0x08, u8, {
    #[doc = "Enables the internal TX data register"]
    enable_data_register @ 7,
    #[doc = "Enables the fifo mode"]
    enable_fifo_mode @ 6,
    #[doc = "Frequency band to use (0: reserved, 1:433, 2:868, 3:915)"]
    freq_band @ 4..6,
    #[doc = "Crystal load capacitance"]
    crystal_load_capacitance @ 0..4,
});

register!(PowerManagement, 0x08, u8, {
    #[doc = "Enables the whole receiver chain (activates RF front end, baseband, synthesizer, crystal oscillator)"]
    enable_receiver_chain @ 7,
    #[doc = "Enables baseband circuit"]
    enable_baseband @ 6,
    #[doc = "Activates sender chain(power amp, synthesizer, crystal oscillator) and starts sending if TX register is enabled"]
    enable_sender_chain @ 5,
    #[doc = "Enables synthesizer"]
    enable_synthesizer @ 4,
    #[doc = "Enables crystal oscillator"]
    enable_oscillator @ 3,
    #[doc = "Enables low battery detector"]
    enable_low_bat_detector @ 2,
    #[doc = "Enables wakeup timer"]
    enable_wakeup_timer @ 1,
    #[doc = "Disable clock output"]
    disable_clock_output @ 0,
});

register!(FrequencySetting, 0x0680, u16, {
    #[doc = "Sets frequency"]
    frequency @ 0..12,
});

register!(DataRate, 0x23, u8, {
    #[doc = "No idea what this is"]
    cs @ 7,
    #[doc = "Data rate"]
    data_rate @ 0..7,
});

register!(RxControl, 0x0080, u16, {
    #[doc = "Pin 16 function (0: interrupt input, 1: VDI input)"]
    p16_func @ 10,
    #[doc = "VDI signal response time (0..3: Fast - Slow, 4: Always on)"]
    vdi_signal_response_time @ 8..10,
    #[doc = "bandwidth of the receiver baseband"]
    receiver_baseband_bw @ 5..8,
    #[doc = "LNA gain select"]
    lna_gain @ 3..5,
    #[doc = "RSSI threshold value"]
    rssi_thresh @ 0..3,
});

register!(DataFilter, 0x2C, u8, {
    #[doc = "Enables clock recovery auto mode(1: auto, 0: use mode selected in bit 6)"]
    auto_clock_recovery_lock @ 7,
    #[doc = "Sets the clock recovery mode (0: slow, 1: fast)"]
    clock_recovery_lock @ 6,
    #[doc = "select filter type (0: internal digital, 1: external analog)"]
    filter_type @ 4,
    #[doc = "DQD threshold"]
    dqd_theshold @ 0..3,
});

register!(FifoAndResetMode, 0x80, u8, {
    #[doc = "FIFO IT level"]
    fifo_it_level @ 4..8,
    #[doc = "synchron pattern length (0: 1b, 1: 2b)"]
    synchron_pattern_len @ 3,
    #[doc = "FIFO fill start condition (0: after synchron pattern, 1; always)"]
    fifo_fill_cond @ 2,
    #[doc = "FIFO will fill until this bit is cleared"]
    enable_fifo_fill @ 1,
    #[doc = "enable sensitive reset mode"]
    sensitive_reset @ 0,
});

register!(SynchPattern, 0xD4, u8, {
    #[doc = "The synchron pattern used by this radio"]
    synchron_pattern @ 0..8,
});

register!(Afc, 0xF7, u8, {
    #[doc = "Automatic operation mode selector"]
    afc_selector @ 6..8,
    #[doc = "Range limit"]
    range_limit @ 4..6,
    #[doc = "Strobe edge"]
    strobe_edge @ 3,
    #[doc = "High accuracy mode"]
    high_acuracy_mode @ 2,
    #[doc = "Apply offset to PLL"]
    enable_frequency_offset @ 1,
    #[doc = "Enable calculation of offset"]
    enable_calculation @ 0,
});

register!(TxConfig, 0x0000, u16, {
    #[doc = "FSK modulation sign (0: +, 1: -)"]
    fsk_mod_sign @ 8,
    #[doc = "FSK modulation output frequency"]
    fsk_mod_freq @ 0..8,
    #[doc = "Output power"]
    output_power @ 0..3,
});

register!(PllSetting, 0x77, u8, {
    #[doc = "Output clock frequency"]
    output_clock_freq @ 5..7,
    #[doc = "Enable delay in phase detector"]
    enable_phase_detec_delay @ 3,
    #[doc = "Disable pll dithering"]
    disable_pll_dithering @ 2,
    #[doc = "PLL bandwidth"]
    pll_bw @ 0,
});

register!(TxRegisterWrite, 0xAA, u8, {
    #[doc = "Writes byte to the transmitter data register"]
    tx_data_register @ 0..8,
});

register!(WakeUpTimer, 0x0196, u16, {
    #[doc = "Wake up periods exponent"]
    wake_up_period_exponent @ 8..13,
    #[doc = "Wake up periods mantissa"]
    wake_up_period_mantissa @ 0..8,
});

register!(LowDutyCycle, 0x0E, u8, {
    #[doc = "duty cycle nominator"]
    duty_cycle_nominator @ 0..8,
});

register!(LowBatDetector, 0x00, u8, {
    #[doc = "Threshold voltage"]
    v_thresh @ 5..8,
    #[doc = "Clock divider for the output clock"]
    cdiv @ 0..4,
});

register!(StatusRead, 0x0000, u32, {
    #[doc = "TX is ready for next byte(TX mode)"]
    tx_ready @ 15,
    #[doc = "RX fifo threshold reached (RX mode)"]
    rx_ready @ 15,
    #[doc = "Power on reset (cleared after first status read)"]
    por @ 14,
    #[doc = "TX register overrun (TX mode)"]
    tx_overrun @ 13,
    #[doc = "RX FIFO underrun (RX mode)"]
    rx_undderrun @ 13,
    #[doc = "Wake-Up timer overflow"]
    wake_up_timer_underflow @ 12,
    #[doc = "External interupt on Pin 16"]
    p16_exti @ 11,
    #[doc = "Low battery detect"]
    low_bat_detec @ 10,
    #[doc = "FIFO empty"]
    fifo_empty @ 9,
    #[doc = "Antenna circuit detected signal above threshold"]
    ant_thresh @ 8,
    #[doc = "RSSI above threshold"]
    rssi_thresh @ 8,
    #[doc = "Data quality detector output"]
    dqd_output @ 7,
    #[doc = "Clock recovery locked"]
    clock_recovery_locked @ 6,
    #[doc = "AFC toggled in each cycle"]
    afc_toggling @ 5,
    #[doc = "frequency offset sign"]
    freq_offset_sign @ 4,
    #[doc = "frequency offset 4 LSBS"]
    freq_offset @ 0..4,
});