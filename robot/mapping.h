#pragma once
#include <stddef.h>

void mappingStart();
void mappingUpdate();
void mappingStop();
bool mappingIsActive();
size_t mappingCopyDisplay(char* out, size_t capacity);