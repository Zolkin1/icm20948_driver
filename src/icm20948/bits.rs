use bitfield::bitfield;

bitfield! {
    /// bitfields of INT_ENABLE register
    pub struct IntEnable(u8);
    impl Debug;
    /// enable wake on fsync interrupt
    pub reg_wof_en, set_reg_wof_en: 7;
    /// enable wake on motion interrupt on interrupt pin 1
    pub wom_int_en, set_wom_int_en: 3;
    /// enable PLL RDY interrupt on interrupt pin 1
    pub pll_rdy_en, set_pll_rdy_en: 2;
    /// enable DMP interrupt on interrupt pin 1
    pub dmp_int1_en, set_dmp_int1_en: 1;
    /// enable i2c master interrupt on interrupt pin 1
    pub i2c_mst_int_en, set_i2c_mst_int_en: 0;
}

bitfield! {
    /// bitfields of INT_PIN_CFG register
    pub struct IntPinCfg(u8);
    impl Debug;
    /// active low logic level for INT1 pin
    pub int1_actl, set_int1_actl: 7;
    /// INT1 pin is open drain
    pub int1_open, set_int1_open: 6;
    /// INT1 pin is held active until cleared
    pub int1_latch_en, set_int1_latch_en: 5;
    /// true = any read operation clears INT_STATUS register
    /// false = only reading INT_STATUS register clear it
    pub int_anyrd_2clear, set_int_anyrd_2clear: 4;
    /// active low logic level for FSYNC pin
    pub actl_fsync, set_actl_fsync: 3;
    /// enable accepting FSYNC pin interrupt
    pub fsync_int_mode_en, set_fsync_int_mode_en: 2;
    /// set i2c master interface pins to bypass mode when i2c master is disabled
    pub bypass_en, set_bypass_en: 1;
}
