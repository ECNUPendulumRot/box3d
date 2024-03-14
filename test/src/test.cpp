
#include "test.hpp"

TestEntry g_test_entries[MAX_TEST] = { {nullptr} };

int g_test_count = 0;


int register_test(const char *category, const char *name, TestCreateFcn *fcn) {
  int index = g_test_count;
  if (index < MAX_TEST) {
	g_test_entries[index] = { category, name, fcn };
	++g_test_count;
	return index;
  }
  return -1;
}
