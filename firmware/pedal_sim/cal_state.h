#pragma once
#include <stdint.h>
enum St : uint8_t { S_IDLE, S_PROBE, S_SETTLE, S_SEEK_HOME, S_BACK_HOME, S_APPR_HOME,
                    S_BACK_HOME2, S_SEEK_END, S_BACK_END, S_APPR_END, S_RETURN, S_DONE };
