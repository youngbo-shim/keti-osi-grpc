#pragma once

static constexpr unsigned int Hash(const char* str){
  return str[0] ? static_cast<unsigned int>(str[0]) + 0xEDB8832Full * Hash(str + 1) : 8603;
}