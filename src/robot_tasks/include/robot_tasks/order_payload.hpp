#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_tasks {

class OrderPayload {
 public:
  bool Has(const std::string& key) const;
  std::string GetString(const std::string& key, const std::string& fallback = "") const;
  int GetInt(const std::string& key, int fallback = 0) const;
  double GetDouble(const std::string& key, double fallback = 0.0) const;
  bool GetBool(const std::string& key, bool fallback = false) const;
  std::vector<std::string> GetStringList(
      const std::string& key, const std::vector<std::string>& fallback = {}) const;

  void SetString(const std::string& key, const std::string& value);
  void SetList(const std::string& key, const std::vector<std::string>& values);

 private:
  std::unordered_map<std::string, std::string> values_;
  std::unordered_map<std::string, std::vector<std::string>> lists_;
};

std::optional<OrderPayload> ParseOrderPayload(const std::string& text, std::string* message);

}  // namespace robot_tasks
