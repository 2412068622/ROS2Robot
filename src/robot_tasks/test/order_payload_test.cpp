#include "robot_tasks/order_payload.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

TEST(OrderPayloadTest, ParseOrderPayload_FlatJsonObject_ReadsScalarsAndArrays) {
  std::string message;

  const auto payload = ParseOrderPayload(
      R"({"pickup_station":"receiving","dropoff_station":"storage_a","station_ids":["receiving","storage_a","packing"],"quantity":3,"start_if_idle":true})",
      &message);

  ASSERT_TRUE(payload.has_value()) << message;
  EXPECT_EQ(payload->GetString("pickup_station"), "receiving");
  EXPECT_EQ(payload->GetString("dropoff_station"), "storage_a");
  EXPECT_EQ(payload->GetInt("quantity"), 3);
  EXPECT_TRUE(payload->GetBool("start_if_idle"));
  EXPECT_EQ(
      payload->GetStringList("station_ids"),
      (std::vector<std::string>{"receiving", "storage_a", "packing"}));
}

TEST(OrderPayloadTest, ParseOrderPayload_KeyValueFormat_ReadsCommaSeparatedLists) {
  std::string message;

  const auto payload = ParseOrderPayload(
      "station_ids=receiving,storage_a,packing;preempt_current=yes;pickup_x=1.25",
      &message);

  ASSERT_TRUE(payload.has_value()) << message;
  EXPECT_EQ(
      payload->GetStringList("station_ids"),
      (std::vector<std::string>{"receiving", "storage_a", "packing"}));
  EXPECT_TRUE(payload->GetBool("preempt_current"));
  EXPECT_DOUBLE_EQ(payload->GetDouble("pickup_x"), 1.25);
}

TEST(OrderPayloadTest, ParseOrderPayload_EmptyPayload_ReturnsEmptyPayload) {
  std::string message;

  const auto payload = ParseOrderPayload("", &message);

  ASSERT_TRUE(payload.has_value()) << message;
  EXPECT_FALSE(payload->Has("pickup_station"));
  EXPECT_EQ(payload->GetString("missing", "fallback"), "fallback");
}

TEST(OrderPayloadTest, ParseOrderPayload_InvalidJson_ReturnsMessage) {
  std::string message;

  const auto payload = ParseOrderPayload(R"({"station_ids":["receiving")", &message);

  EXPECT_FALSE(payload.has_value());
  EXPECT_NE(message.find("expected ',' or ']'"), std::string::npos);
}

TEST(OrderPayloadTest, ParseOrderPayload_KeyValueWithoutEquals_ReturnsMessage) {
  std::string message;

  const auto payload = ParseOrderPayload("pickup_station", &message);

  EXPECT_FALSE(payload.has_value());
  EXPECT_EQ(message, "payload entry missing '=': pickup_station");
}

}  // namespace
}  // namespace robot_tasks
