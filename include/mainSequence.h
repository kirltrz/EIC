#pragma once
#include"taskManager.h"
#include"motion.h"

extern struct POS pos[10];
void startMainSequence(void);
void mainSequenceTask(void *);