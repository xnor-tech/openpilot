#pragma once

#include <stdbool.h>

// F4-compatible power-saving declarations aligned with xnor-dev's bool API.
extern bool power_save_enabled;
#ifdef ALLOW_DEBUG
extern volatile bool stop_mode_requested;
#endif

void set_power_save_state(bool enable);
