#pragma once

#include <array>


template <typename TVALUE, int NVALS, int FLOAT_PRECISION=6>
class RTDebugOutput {
private:
  std::array<String,NVALS> _outNames;
  
public:
  RTDebugOutput(std::array<String,NVALS> outNames = {})
    : _outNames(outNames) { }

  void offerData(std::array<TVALUE,NVALS> values) {
    printData(values);
  }
  

  template <typename T>
  void printValue(String name, T value) {
    if (name.length() > 0) {
      Serial.print(">"); Serial.print(name); Serial.print(":");
    }
    Serial.print(value); Serial.print("\n");
  }
  void printValue(String name, float value) {
    if (name.length() > 0) {
      Serial.print(">"); Serial.print(name); Serial.print(":"); 
    }
    Serial.print(value, FLOAT_PRECISION); Serial.print("\n");
  }

  void printData(std::array<TVALUE,NVALS> &values) {
    for (int i=0; i<NVALS; i++) {
      printValue(_outNames[i], values[i]);
    }
  }
};
