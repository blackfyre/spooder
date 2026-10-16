#include "reporting.h"

void print_value(String name, float value, bool newLine){
  Serial.print(name + ": ");
  if(newLine)Serial.println(value);
  else Serial.print(value);
}

void print_value(String name, String value, bool newLine){
  Serial.print(name + ": ");
  if(newLine)Serial.println(value);
  else Serial.print(value);
}

String vectorToString(Vector3 value){
  return "(" + String(value.x) + "," + String(value.y) + "," + String(value.z) + ")";
}

void print_value(String name, Vector3 value, bool newLine){
  Serial.print(name + ": ");
  if(newLine)Serial.println(vectorToString(value));
  else Serial.print(vectorToString(value));
}