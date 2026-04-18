#pragma once

// FreeRTOS task entry point for the safety-critical shift owner. The GPIO ISR
// wakes this task, then this task performs the full shift sequence.
void gear_safety_task(void *arg);
