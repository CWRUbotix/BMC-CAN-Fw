/// Enum representing idle behavior
#[derive(Eq, PartialEq, Copy, Clone, defmt::Format)]
pub enum IdleMode {
    Brake,
    Coast,
}
