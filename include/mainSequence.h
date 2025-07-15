#pragma once
#include"taskManager.h"
#include"motion.h"
#include"devMonitor.h"

extern struct POS pos[10];
extern int taskcode[2][3];

void startMainSequence(void);
void mainSequenceTask(void *);
void updateTaskcodeDisplay(void);