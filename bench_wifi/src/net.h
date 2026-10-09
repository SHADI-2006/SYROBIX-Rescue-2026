/**
 * Wi-Fi link for the bench build: HTTP server for the UI (port 80) and a
 * WebSocket (port 81) that carries the same text protocol as USB serial.
 * Everything runs from loop() via netService(); nothing here is thread-safe.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

using NetTextHandler  = void (*)(char* line);   // one command line, NUL-terminated, writable
using NetEventHandler = void (*)();

/** Start Wi-Fi (AP or STA, see wifi_config.h), HTTP and WebSocket servers. */
void netBegin(NetTextHandler onText, NetEventHandler onAllClientsGone);
/** Pump HTTP + WebSocket. Call every loop pass. */
void netService();
/** Broadcast one line (no newline needed) to every WebSocket client. */
void netSend(const char* data, size_t len);
uint8_t     netClients();
const char* netIp();      // "192.168.4.1" or the STA address
const char* netMode();    // "ap" or "sta"
