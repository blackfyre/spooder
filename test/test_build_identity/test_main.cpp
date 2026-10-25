#include <unity.h>
#include <string.h>
#include "identity/build_identity.h"

using namespace build_identity;

void setUp() {}
void tearDown() {}

static char line[BANNER_SIZE];

void test_banner_form()
{
  int n = banner(line, sizeof line, "robot", "077f0e330ca0-dirty");
  TEST_ASSERT_EQUAL_STRING("spooder robot: build 077f0e330ca0-dirty\n", line);
  TEST_ASSERT_EQUAL_INT((int)strlen(line), n);
  banner(line, sizeof line, "bridge", "v1-3-g077f0e330ca0");
  TEST_ASSERT_EQUAL_STRING("spooder bridge: build v1-3-g077f0e330ca0\n", line);
}

void test_empty_and_null_id_are_unknown()
{
  banner(line, sizeof line, "robot", "");
  TEST_ASSERT_EQUAL_STRING("spooder robot: build unknown\n", line);
  banner(line, sizeof line, "robot", nullptr);
  TEST_ASSERT_EQUAL_STRING("spooder robot: build unknown\n", line);
}

void test_non_printable_bytes_become_question_marks()
{
  // 0xA5 is the link's frame start; a tab, a newline, DEL and a UTF-8 byte.
  banner(line, sizeof line, "robot",
         "a\xA5"
         "b\tc\nd\x7F"
         "e\xC3\xA9");
  TEST_ASSERT_EQUAL_STRING("spooder robot: build a?b?c?d?e??\n", line);
  for (const char *p = line; *p != '\0'; p++)
  {
    unsigned char b = (unsigned char)*p;
    TEST_ASSERT_TRUE(b == '\n' || (b >= 0x20 && b <= 0x7E));
  }
}

void test_banner_cuts_the_id_at_40_characters()
{
  const char *id = "v\"1-quoted-annotated-tag-name-123-g077f0e330ca0-dirty"; // 53
  banner(line, sizeof line, "bridge", id);
  TEST_ASSERT_EQUAL_STRING("spooder bridge: build v\"1-quoted-annotated-tag-name-123-g077f0\n", line);
  TEST_ASSERT_EQUAL_INT(MAX_ID_CHARS, (int)strlen(line) - (int)strlen("spooder bridge: build \n"));
  TEST_ASSERT_TRUE(strlen(line) < (size_t)BANNER_SIZE); // the longest banner fits
  // Exactly 40 is kept whole.
  banner(line, sizeof line, "robot", "0123456789012345678901234567890123456789");
  TEST_ASSERT_EQUAL_STRING("spooder robot: build 0123456789012345678901234567890123456789\n", line);
}

void test_banner_into_a_small_buffer_is_cut_and_terminated()
{
  char small[12];
  memset(small, 'x', sizeof small);
  int n = banner(small, sizeof small, "robot", "abc");
  TEST_ASSERT_EQUAL_STRING("spooder rob", small);
  TEST_ASSERT_EQUAL_INT(11, n);
}

void test_fit_keeps_an_id_that_fits()
{
  char out[23];
  fit(out, 22, "077f0e330ca0-dirty");
  TEST_ASSERT_EQUAL_STRING("077f0e330ca0-dirty", out);
  fit(out, 22, "0123456789012345678901"); // exactly 22
  TEST_ASSERT_EQUAL_STRING("0123456789012345678901", out);
  fit(out, 22, "");
  TEST_ASSERT_EQUAL_STRING("unknown", out);
  fit(out, 22, nullptr);
  TEST_ASSERT_EQUAL_STRING("unknown", out);
}

void test_fit_keeps_the_end_of_a_long_id()
{
  char out[23];
  fit(out, 22, "v1.0-rc1-15-g077f0e330ca0-dirty"); // 31
  TEST_ASSERT_EQUAL_STRING("..-g077f0e330ca0-dirty", out);
  TEST_ASSERT_EQUAL_INT(22, (int)strlen(out));
  // Longer than the banner's 40: the end is still shown.
  fit(out, 22, "a-very-long-annotated-tag-name-1-g077f0e330ca0-dirty");
  TEST_ASSERT_EQUAL_STRING("..-g077f0e330ca0-dirty", out);
  fit(out, 22, "0123456789012345678901\xA5"); // 23, the last byte replaced
  TEST_ASSERT_EQUAL_STRING("..3456789012345678901?", out);
}

void test_fit_narrow_widths()
{
  char out[4];
  fit(out, 3, "abcdef");
  TEST_ASSERT_EQUAL_STRING("..f", out);
  fit(out, 2, "abcdef");
  TEST_ASSERT_EQUAL_STRING("..", out);
  fit(out, 0, "abcdef");
  TEST_ASSERT_EQUAL_STRING("", out);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_banner_form);
  RUN_TEST(test_empty_and_null_id_are_unknown);
  RUN_TEST(test_non_printable_bytes_become_question_marks);
  RUN_TEST(test_banner_cuts_the_id_at_40_characters);
  RUN_TEST(test_banner_into_a_small_buffer_is_cut_and_terminated);
  RUN_TEST(test_fit_keeps_an_id_that_fits);
  RUN_TEST(test_fit_keeps_the_end_of_a_long_id);
  RUN_TEST(test_fit_narrow_widths);
  return UNITY_END();
}
