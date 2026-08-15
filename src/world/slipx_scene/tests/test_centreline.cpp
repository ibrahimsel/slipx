// Copyright 2026 The SlipX Authors
// SPDX-License-Identifier: Apache-2.0
//
// The centreline parser.
//
// Half of these cases are refusals. That ratio is the point: the format has
// no way to say "this field is missing", so everything the loader will not
// invent has to come back as an error naming a line somebody can open.

#include <gtest/gtest.h>

#include <locale>
#include <stdexcept>
#include <string>

#include "slipx/scene/centreline.hpp"

namespace {

using slipx::scene::Centreline;

// The message of the std::invalid_argument a call throws, or "" if it did not
// throw. The refusals are meant to be readable by a person with the file
// open, so the tests read them.
template <typename F>
std::string refusal_from(F&& call) {
  try {
    call();
  } catch (const std::invalid_argument& e) {
    return e.what();
  }
  return std::string();
}

bool mentions(const std::string& haystack, const std::string& needle) {
  return haystack.find(needle) != std::string::npos;
}

constexpr const char* kOrigin = "porto/centreline.csv";

TEST(Centreline, ParsesTheFourColumnTumForm) {
  const Centreline track = Centreline::from_csv(
      "# x_m,y_m,w_tr_right_m,w_tr_left_m\n"
      "0.0,0.0,1.5,2.5\n"
      "3.0,0.0,1.5,2.5\n",
      kOrigin);

  ASSERT_EQ(track.size(), 2u);
  EXPECT_DOUBLE_EQ(track.points()[0].x, 0.0);
  EXPECT_DOUBLE_EQ(track.points()[1].x, 3.0);
  EXPECT_EQ(track.origin(), kOrigin);
}

// The one column-order mistake that is invisible on a symmetric test file.
// The widths here differ so that a swap changes the answer.
TEST(Centreline, RightWidthComesBeforeLeftWidth) {
  const Centreline track = Centreline::from_csv(
      "0.0,0.0,1.0,9.0\n"
      "1.0,0.0,1.0,9.0\n",
      kOrigin);

  EXPECT_DOUBLE_EQ(track.points()[0].w_right, 1.0);
  EXPECT_DOUBLE_EQ(track.points()[0].w_left, 9.0);
}

TEST(Centreline, DerivesArcLengthInFileOrder) {
  // A 3-4-5 triangle, so the cumulative lengths are exact in binary.
  const Centreline track = Centreline::from_csv(
      "0.0,0.0,1.0,1.0\n"
      "3.0,0.0,1.0,1.0\n"
      "3.0,4.0,1.0,1.0\n",
      kOrigin);

  EXPECT_DOUBLE_EQ(track.points()[0].s, 0.0);
  EXPECT_DOUBLE_EQ(track.points()[1].s, 3.0);
  EXPECT_DOUBLE_EQ(track.points()[2].s, 7.0);
  EXPECT_DOUBLE_EQ(track.open_length(), 7.0);
  // The two legs are 3 and 4, so the chord back to the start is the 5.
  EXPECT_DOUBLE_EQ(track.closing_chord(), 5.0);
}

TEST(Centreline, SkipsCommentsBlankLinesAndCarriageReturns) {
  const Centreline track = Centreline::from_csv(
      "# a header\r\n"
      "\r\n"
      "0.0,0.0,1.0,1.0\r\n"
      "   \n"
      "  # an indented comment\n"
      " 1.0 , 0.0 , 1.0 , 1.0 \n",
      kOrigin);

  ASSERT_EQ(track.size(), 2u);
  EXPECT_DOUBLE_EQ(track.points()[1].x, 1.0);
}

// The trap this file was written around. A machine whose locale uses a comma
// decimal separator must read 1.5 as 1.5, not as 1, and not as 15.
TEST(Centreline, ParsesUnderACommaDecimalLocale) {
  struct CommaDecimal : std::numpunct<char> {
   protected:
    char do_decimal_point() const override { return ','; }
  };

  // Restored on every path out, including a failed assertion, because a
  // global locale left behind would silently change every later test.
  struct Restore {
    std::locale previous;
    ~Restore() { std::locale::global(previous); }
  } restore{std::locale()};

  std::locale::global(std::locale(std::locale::classic(), new CommaDecimal));

  const Centreline track = Centreline::from_csv(
      "0.0,0.0,1.5,1.5\n"
      "2.5,0.0,1.5,1.5\n",
      kOrigin);

  EXPECT_DOUBLE_EQ(track.points()[0].w_right, 1.5);
  EXPECT_DOUBLE_EQ(track.points()[1].x, 2.5);
  EXPECT_DOUBLE_EQ(track.open_length(), 2.5);
}

TEST(Centreline, RefusesAWrongFieldCount) {
  const std::string few = refusal_from([] {
    Centreline::from_csv("0.0,0.0,1.0,1.0\n1.0,0.0,1.0\n", kOrigin);
  });
  EXPECT_TRUE(mentions(few, "porto/centreline.csv:2")) << few;
  EXPECT_TRUE(mentions(few, "found 3")) << few;

  const std::string many = refusal_from([] {
    Centreline::from_csv("0.0,0.0,1.0,1.0,0.0\n1.0,0.0,1.0,1.0\n", kOrigin);
  });
  EXPECT_TRUE(mentions(many, "porto/centreline.csv:1")) << many;
  EXPECT_TRUE(mentions(many, "found 5")) << many;
}

TEST(Centreline, RefusesAFieldThatIsNotANumber) {
  const std::string message = refusal_from([] {
    Centreline::from_csv("0.0,0.0,1.0,1.0\n1.0,wide,1.0,1.0\n", kOrigin);
  });
  EXPECT_TRUE(mentions(message, "porto/centreline.csv:2")) << message;
  EXPECT_TRUE(mentions(message, "y_m")) << message;
}

// "1.5abc" must not read as 1.5. A parser that stops at the first character
// it does not understand turns a corrupted file into a plausible track.
TEST(Centreline, RefusesTrailingRubbishInAField) {
  const std::string message = refusal_from([] {
    Centreline::from_csv("0.0,0.0,1.0,1.0\n1.5abc,0.0,1.0,1.0\n", kOrigin);
  });
  EXPECT_TRUE(mentions(message, "x_m")) << message;
}

// Which of the two refusals fires depends on the standard library and is not
// worth pinning. libstdc++ rejects these spellings in the number grammar, so
// they come back as "is not a number"; libc++ accepts them, and then the
// finiteness check in the loader is what stops them. The behaviour that
// matters is that neither ever reaches a track, and that the message names
// the field and the line either way.
TEST(Centreline, RefusesANonFiniteField) {
  for (const char* value : {"nan", "inf", "-inf", "1e400"}) {
    const std::string text =
        std::string("0.0,0.0,1.0,1.0\n") + value + ",0.0,1.0,1.0\n";
    const std::string message =
        refusal_from([&text] { Centreline::from_csv(text, kOrigin); });
    EXPECT_TRUE(mentions(message, "x_m")) << value << ": " << message;
    EXPECT_TRUE(mentions(message, "porto/centreline.csv:2"))
        << value << ": " << message;
    EXPECT_TRUE(mentions(message, "not finite") ||
                mentions(message, "not a number"))
        << value << ": " << message;
  }
}

// Every field here is a finite double and the distance between them is not.
// This is the case that reaches the finiteness guard on a library whose
// number grammar accepts the fields above, and the one that justifies the
// loader computing distance the cheap, correctly rounded way.
TEST(Centreline, RefusesACentrelineWhoseArcLengthOverflows) {
  const std::string message = refusal_from([] {
    Centreline::from_csv(
        "-1e308,0.0,1.0,1.0\n"
        "1e308,0.0,1.0,1.0\n",
        kOrigin);
  });
  EXPECT_TRUE(mentions(message, "arc length is not finite")) << message;
  EXPECT_TRUE(mentions(message, "line 2")) << message;
}

TEST(Centreline, RefusesANonPositiveWidth) {
  const std::string zero = refusal_from([] {
    Centreline::from_csv("0.0,0.0,0.0,1.0\n1.0,0.0,1.0,1.0\n", kOrigin);
  });
  EXPECT_TRUE(mentions(zero, "porto/centreline.csv:1")) << zero;
  EXPECT_TRUE(mentions(zero, "positive")) << zero;

  const std::string negative = refusal_from([] {
    Centreline::from_csv("0.0,0.0,1.0,-1.0\n1.0,0.0,1.0,1.0\n", kOrigin);
  });
  EXPECT_TRUE(mentions(negative, "positive")) << negative;
}

TEST(Centreline, RefusesFewerThanTwoPoints) {
  const std::string header_only = refusal_from([] {
    Centreline::from_csv("# x_m,y_m,w_tr_right_m,w_tr_left_m\n", kOrigin);
  });
  EXPECT_TRUE(mentions(header_only, "at least 2 points")) << header_only;

  const std::string single = refusal_from(
      [] { Centreline::from_csv("0.0,0.0,1.0,1.0\n", kOrigin); });
  EXPECT_TRUE(mentions(single, "found 1")) << single;
}

// A zero-length segment has no direction, and the first thing that wants one
// is lap counting. It is refused here, where the line numbers still exist.
TEST(Centreline, RefusesTwoConsecutivePointsAtTheSamePosition) {
  const std::string message = refusal_from([] {
    Centreline::from_csv(
        "# header\n"
        "0.0,0.0,1.0,1.0\n"
        "1.0,0.0,1.0,1.0\n"
        "1.0,0.0,1.0,2.0\n",
        kOrigin);
  });
  EXPECT_TRUE(mentions(message, "lines 3 and 4")) << message;
}

// The same position twice is only refused when the points are adjacent: a
// closed lap revisits its start, and a figure of eight crosses itself.
TEST(Centreline, AllowsAPositionThatRepeatsNonAdjacently) {
  EXPECT_NO_THROW({
    Centreline::from_csv(
        "0.0,0.0,1.0,1.0\n"
        "1.0,0.0,1.0,1.0\n"
        "0.0,0.0,1.0,1.0\n",
        kOrigin);
  });
}

TEST(Centreline, FromFileNamesThePathWhenItCannotBeOpened) {
  const std::string message = refusal_from(
      [] { Centreline::from_file("no/such/track/centreline.csv"); });
  EXPECT_TRUE(mentions(message, "no/such/track/centreline.csv")) << message;
}

}  // namespace
