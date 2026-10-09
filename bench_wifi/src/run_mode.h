/**
 * RUN mode: boots the REAL match firmware (../src) exactly like its main.cpp,
 * and adds a Wi-Fi remote: virtual START, STOP (TaskHandler::emergencyStop),
 * live mission state. Practice only — never for a scored round.
 */
#pragma once

void runSetup();
void runLoop();
