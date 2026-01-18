use crate::itedtv_bus::BusOps;
use crate::tc90522::TC90522;
use crate::rt710::RT710;
use crate::r850::R850;

#[derive(Debug, Clone, Copy)]
pub enum System
{
    ISDB_S,
    ISDB_T,
}

pub enum Tuner<'a, B: BusOps>
{
    RT710(RT710<'a, B>),
    R850(R850<'a, B>),
}

pub struct Px4Chrdev<'a, B: BusOps>
{
    pub system: System,

    pub port_number: u8,
    pub slave_number: u8,
    pub sync_byte: u8,

    pub tc90522: TC90522<'a, B>,
    pub tuner: Tuner<'a, B>,
}

pub struct Px4Device<'a, B: BusOps>
{
    pub px4chrdev: Vec<Px4Chrdev<'a, B>>,
}