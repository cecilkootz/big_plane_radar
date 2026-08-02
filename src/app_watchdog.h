#pragma once

namespace AppWatchdog {

void logResetReason();
bool begin();
bool subscribeCurrentTask(const char *name);
void feed();

}  // namespace AppWatchdog
