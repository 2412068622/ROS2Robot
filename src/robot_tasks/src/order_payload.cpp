#include "robot_tasks/order_payload.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <stdexcept>

namespace robot_tasks {
namespace {

std::string Trim(const std::string& value) {
  std::size_t begin = 0;
  while (begin < value.size() &&
         std::isspace(static_cast<unsigned char>(value[begin])) != 0) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(value[end - 1])) != 0) {
    --end;
  }
  return value.substr(begin, end - begin);
}

std::string NormalizeKey(const std::string& value) {
  std::string key = Trim(value);
  std::transform(key.begin(), key.end(), key.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return key;
}

std::vector<std::string> SplitCommaList(const std::string& value) {
  std::vector<std::string> parts;
  std::string current;
  bool in_quote = false;
  for (std::size_t index = 0; index < value.size(); ++index) {
    const char ch = value[index];
    if (ch == '"' && (index == 0 || value[index - 1] != '\\')) {
      in_quote = !in_quote;
      current.push_back(ch);
      continue;
    }
    if (ch == ',' && !in_quote) {
      parts.push_back(Trim(current));
      current.clear();
      continue;
    }
    current.push_back(ch);
  }
  if (!current.empty() || !value.empty()) {
    parts.push_back(Trim(current));
  }
  return parts;
}

std::string Unquote(const std::string& value) {
  const auto trimmed = Trim(value);
  if (trimmed.size() < 2U || trimmed.front() != '"' || trimmed.back() != '"') {
    return trimmed;
  }
  std::string result;
  result.reserve(trimmed.size() - 2U);
  for (std::size_t index = 1; index + 1U < trimmed.size(); ++index) {
    const char ch = trimmed[index];
    if (ch != '\\' || index + 2U >= trimmed.size()) {
      result.push_back(ch);
      continue;
    }
    const char escaped = trimmed[++index];
    switch (escaped) {
      case '"':
      case '\\':
      case '/':
        result.push_back(escaped);
        break;
      case 'n':
        result.push_back('\n');
        break;
      case 'r':
        result.push_back('\r');
        break;
      case 't':
        result.push_back('\t');
        break;
      default:
        result.push_back(escaped);
        break;
    }
  }
  return result;
}

class FlatJsonParser {
 public:
  explicit FlatJsonParser(const std::string& text) : text_(text) {}

  std::optional<OrderPayload> Parse(std::string* message) {
    OrderPayload payload;
    SkipWhitespace();
    if (!Consume('{')) {
      *message = "payload_json must start with '{'";
      return std::nullopt;
    }
    SkipWhitespace();
    if (Consume('}')) {
      return payload;
    }
    while (position_ < text_.size()) {
      std::string key;
      if (!ParseString(&key, message)) {
        return std::nullopt;
      }
      key = NormalizeKey(key);
      SkipWhitespace();
      if (!Consume(':')) {
        *message = "expected ':' after key '" + key + "'";
        return std::nullopt;
      }
      SkipWhitespace();
      if (Peek() == '[') {
        std::vector<std::string> values;
        if (!ParseArray(&values, message)) {
          return std::nullopt;
        }
        payload.SetList(key, values);
      } else {
        std::string value;
        if (!ParseScalar(&value, message)) {
          return std::nullopt;
        }
        payload.SetString(key, value);
      }
      SkipWhitespace();
      if (Consume('}')) {
        SkipWhitespace();
        if (position_ != text_.size()) {
          *message = "unexpected trailing content after JSON object";
          return std::nullopt;
        }
        return payload;
      }
      if (!Consume(',')) {
        *message = "expected ',' or '}' in payload_json";
        return std::nullopt;
      }
      SkipWhitespace();
    }
    *message = "unterminated payload_json object";
    return std::nullopt;
  }

 private:
  char Peek() const {
    return position_ < text_.size() ? text_[position_] : '\0';
  }

  void SkipWhitespace() {
    while (position_ < text_.size() &&
           std::isspace(static_cast<unsigned char>(text_[position_])) != 0) {
      ++position_;
    }
  }

  bool Consume(const char ch) {
    SkipWhitespace();
    if (Peek() != ch) {
      return false;
    }
    ++position_;
    return true;
  }

  bool ParseString(std::string* value, std::string* message) {
    SkipWhitespace();
    if (Peek() != '"') {
      *message = "expected JSON string";
      return false;
    }
    ++position_;
    std::string result;
    while (position_ < text_.size()) {
      const char ch = text_[position_++];
      if (ch == '"') {
        *value = result;
        return true;
      }
      if (ch != '\\') {
        result.push_back(ch);
        continue;
      }
      if (position_ >= text_.size()) {
        *message = "unterminated escape sequence";
        return false;
      }
      const char escaped = text_[position_++];
      switch (escaped) {
        case '"':
        case '\\':
        case '/':
          result.push_back(escaped);
          break;
        case 'n':
          result.push_back('\n');
          break;
        case 'r':
          result.push_back('\r');
          break;
        case 't':
          result.push_back('\t');
          break;
        default:
          result.push_back(escaped);
          break;
      }
    }
    *message = "unterminated JSON string";
    return false;
  }

  bool ParseArray(std::vector<std::string>* values, std::string* message) {
    if (!Consume('[')) {
      *message = "expected '['";
      return false;
    }
    SkipWhitespace();
    if (Consume(']')) {
      return true;
    }
    while (position_ < text_.size()) {
      std::string value;
      if (!ParseScalar(&value, message)) {
        return false;
      }
      values->push_back(value);
      SkipWhitespace();
      if (Consume(']')) {
        return true;
      }
      if (!Consume(',')) {
        *message = "expected ',' or ']' in JSON array";
        return false;
      }
      SkipWhitespace();
    }
    *message = "unterminated JSON array";
    return false;
  }

  bool ParseScalar(std::string* value, std::string* message) {
    SkipWhitespace();
    if (Peek() == '"') {
      return ParseString(value, message);
    }
    const std::size_t begin = position_;
    while (position_ < text_.size()) {
      const char ch = text_[position_];
      if (ch == ',' || ch == '}' || ch == ']') {
        break;
      }
      ++position_;
    }
    const auto token = Trim(text_.substr(begin, position_ - begin));
    if (token.empty()) {
      *message = "empty JSON value";
      return false;
    }
    *value = token == "null" ? "" : token;
    return true;
  }

  const std::string& text_;
  std::size_t position_{0};
};

std::optional<OrderPayload> ParseKeyValuePayload(
    const std::string& text, std::string* message) {
  OrderPayload payload;
  std::size_t begin = 0;
  while (begin <= text.size()) {
    const std::size_t end = text.find_first_of(";\n", begin);
    const auto token =
        Trim(text.substr(begin, end == std::string::npos ? std::string::npos : end - begin));
    if (!token.empty()) {
      const auto equals = token.find('=');
      if (equals == std::string::npos) {
        *message = "payload entry missing '=': " + token;
        return std::nullopt;
      }
      const auto key = NormalizeKey(token.substr(0, equals));
      const auto raw_value = Trim(token.substr(equals + 1U));
      if (key.empty()) {
        *message = "payload entry has empty key";
        return std::nullopt;
      }
      if (raw_value.size() >= 2U && raw_value.front() == '[' && raw_value.back() == ']') {
        std::vector<std::string> values;
        for (const auto& part : SplitCommaList(raw_value.substr(1U, raw_value.size() - 2U))) {
          values.push_back(Unquote(part));
        }
        payload.SetList(key, values);
      } else if (raw_value.find(',') != std::string::npos) {
        std::vector<std::string> values;
        for (const auto& part : SplitCommaList(raw_value)) {
          values.push_back(Unquote(part));
        }
        payload.SetList(key, values);
      } else {
        payload.SetString(key, Unquote(raw_value));
      }
    }
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return payload;
}

bool ToBool(const std::string& value, bool fallback) {
  auto normalized = NormalizeKey(value);
  if (normalized == "true" || normalized == "1" || normalized == "yes" ||
      normalized == "on") {
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" ||
      normalized == "off") {
    return false;
  }
  return fallback;
}

}  // namespace

bool OrderPayload::Has(const std::string& key) const {
  const auto normalized = NormalizeKey(key);
  return values_.find(normalized) != values_.end() || lists_.find(normalized) != lists_.end();
}

std::string OrderPayload::GetString(
    const std::string& key, const std::string& fallback) const {
  const auto it = values_.find(NormalizeKey(key));
  return it == values_.end() ? fallback : it->second;
}

int OrderPayload::GetInt(const std::string& key, const int fallback) const {
  const auto value = GetString(key);
  if (value.empty()) {
    return fallback;
  }
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    return fallback;
  }
}

double OrderPayload::GetDouble(const std::string& key, const double fallback) const {
  const auto value = GetString(key);
  if (value.empty()) {
    return fallback;
  }
  char* end = nullptr;
  const double result = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0') {
    return fallback;
  }
  return result;
}

bool OrderPayload::GetBool(const std::string& key, const bool fallback) const {
  const auto value = GetString(key);
  return value.empty() ? fallback : ToBool(value, fallback);
}

std::vector<std::string> OrderPayload::GetStringList(
    const std::string& key, const std::vector<std::string>& fallback) const {
  const auto list_it = lists_.find(NormalizeKey(key));
  if (list_it != lists_.end()) {
    return list_it->second;
  }
  const auto value_it = values_.find(NormalizeKey(key));
  if (value_it == values_.end()) {
    return fallback;
  }
  std::vector<std::string> values;
  for (const auto& part : SplitCommaList(value_it->second)) {
    values.push_back(Unquote(part));
  }
  return values.empty() ? fallback : values;
}

void OrderPayload::SetString(const std::string& key, const std::string& value) {
  values_[NormalizeKey(key)] = value;
  lists_.erase(NormalizeKey(key));
}

void OrderPayload::SetList(const std::string& key, const std::vector<std::string>& values) {
  lists_[NormalizeKey(key)] = values;
  values_.erase(NormalizeKey(key));
}

std::optional<OrderPayload> ParseOrderPayload(
    const std::string& text, std::string* message) {
  const auto trimmed = Trim(text);
  if (trimmed.empty()) {
    return OrderPayload{};
  }
  if (trimmed.front() == '{') {
    return FlatJsonParser(trimmed).Parse(message);
  }
  return ParseKeyValuePayload(trimmed, message);
}

}  // namespace robot_tasks
