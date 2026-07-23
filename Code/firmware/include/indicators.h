#pragma once

// ============================================================================
// LED (WS2812) + buzzer status indication. Self-contained: reads
// flight_state.h and esc_telemetry.h internally rather than taking arguments,
// since it's purely an observer of flight state, never a decision-maker.
// ============================================================================

void indicatorsInit();

// Call periodically (INDICATOR_TASK_HZ, ~50Hz, is plenty) from a low-priority task.
void indicatorsUpdate();
