#ifndef REPORTING_H
#define REPORTING_H

#include <Arduino.h>
#include "../movement/vectors.h"

void print_value(String name, float value, bool newLine);

void print_value(String name, String value, bool newLine);

void print_value(String name, Vector3 value, bool newLine);

#endif