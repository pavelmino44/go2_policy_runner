// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.
// Copyright (c) 2026, unitree-go2-rl contributors.
// SPDX-License-Identifier: Apache-2.0
//
// This file is a modified version of code from unitree_rl_lab
// (https://github.com/unitreerobotics/unitree_rl_lab, Apache License 2.0):
//   deploy/include/unitree_joystick_dsl.hpp
// Changes by unitree-go2-rl contributors (2026):
//   - decimal hold times
//   - key lookup resolved when the expression is compiled
//   - errors reported as std::invalid_argument with the full expression

#include "go2_policy_runner/remote/remote_dsl.hpp"

#include <cctype>
#include <memory>
#include <stdexcept>
#include <utility>

namespace go2_policy_runner
{

namespace
{

struct Token
{
  enum Kind { kIdent, kNumber, kPlus, kOr, kNot, kLParen, kRParen, kDot, kEnd } kind;
  std::string text;
};

class Lexer
{
public:
  explicit Lexer(const std::string & s)
  : s_(s) {}

  Token next()
  {
    while (pos_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[pos_]))) {++pos_;}
    if (pos_ >= s_.size()) {return {Token::kEnd, ""};}
    const char c = s_[pos_];
    if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
      const std::size_t start = pos_;
      while (pos_ < s_.size() &&
        (std::isalnum(static_cast<unsigned char>(s_[pos_])) || s_[pos_] == '_'))
      {
        ++pos_;
      }
      return {Token::kIdent, s_.substr(start, pos_ - start)};
    }
    if (std::isdigit(static_cast<unsigned char>(c))) {
      const std::size_t start = pos_;
      bool dot = false;
      while (pos_ < s_.size() &&
        (std::isdigit(static_cast<unsigned char>(s_[pos_])) || (!dot && s_[pos_] == '.')))
      {
        dot = dot || s_[pos_] == '.';
        ++pos_;
      }
      return {Token::kNumber, s_.substr(start, pos_ - start)};
    }
    ++pos_;
    switch (c) {
      case '+': return {Token::kPlus, "+"};
      case '|': return {Token::kOr, "|"};
      case '!': return {Token::kNot, "!"};
      case '(': return {Token::kLParen, "("};
      case ')': return {Token::kRParen, ")"};
      case '.': return {Token::kDot, "."};
      default:
        throw std::invalid_argument(
                "unexpected character '" + std::string(1, c) + "' at position " +
                std::to_string(pos_ - 1));
    }
  }

  std::size_t pos() const {return pos_;}

private:
  const std::string s_;
  std::size_t pos_{0};
};

enum class Field { kPressed, kOnPressed, kOnReleased, kHeldFor };

class Parser
{
public:
  explicit Parser(const std::string & expr)
  : lex_(expr) {tok_ = lex_.next();}

  RemoteCondition parse()
  {
    auto c = parse_or();
    if (tok_.kind != Token::kEnd) {fail("trailing tokens");}
    return c;
  }

private:
  RemoteCondition parse_or()
  {
    auto left = parse_and();
    while (tok_.kind == Token::kOr) {
      eat(Token::kOr);
      auto right = parse_and();
      left = [l = std::move(left), r = std::move(right)](const UnitreeRemote & j) {
          return l(j) || r(j);
        };
    }
    return left;
  }

  RemoteCondition parse_and()
  {
    auto left = parse_unary();
    while (tok_.kind == Token::kPlus) {
      eat(Token::kPlus);
      auto right = parse_unary();
      left = [l = std::move(left), r = std::move(right)](const UnitreeRemote & j) {
          return l(j) && r(j);
        };
    }
    return left;
  }

  RemoteCondition parse_unary()
  {
    if (tok_.kind == Token::kNot) {
      eat(Token::kNot);
      auto child = parse_unary();
      return [c = std::move(child)](const UnitreeRemote & j) {return !c(j);};
    }
    if (tok_.kind == Token::kLParen) {
      eat(Token::kLParen);
      auto inside = parse_or();
      eat(Token::kRParen);
      return inside;
    }
    return parse_atom();
  }

  RemoteCondition parse_atom()
  {
    if (tok_.kind != Token::kIdent) {fail("expected key name");}
    const std::string name = tok_.text;
    // Resolve the key once, so typos in the config fail at startup and evaluation is cheap.
    const UnitreeRemote::KeyGetter getter = UnitreeRemote::find_key(name);
    if (!getter) {fail("unknown key '" + name + "'");}
    eat(Token::kIdent);

    Field field = Field::kPressed;
    float hold = 0.0f;
    if (tok_.kind == Token::kLParen) {
      eat(Token::kLParen);
      if (tok_.kind != Token::kNumber) {fail("expected hold time");}
      hold = std::stof(tok_.text);
      eat(Token::kNumber);
      if (tok_.kind == Token::kIdent) {
        const std::string unit = tok_.text;
        if (unit != "s" && unit != "sec" && unit != "secs") {
          fail("unknown time unit '" + unit + "' (use s)");
        }
        eat(Token::kIdent);
      }
      eat(Token::kRParen);
      field = Field::kHeldFor;
    }
    if (tok_.kind == Token::kDot) {
      eat(Token::kDot);
      if (tok_.kind != Token::kIdent) {fail("expected pressed|on_pressed|on_released");}
      const std::string st = tok_.text;
      if (st == "on_pressed") {
        field = Field::kOnPressed;
      } else if (st == "on_released") {
        field = Field::kOnReleased;
      } else if (st == "pressed") {
        // "LT(2s).pressed" keeps the hold requirement
        if (field != Field::kHeldFor) {field = Field::kPressed;}
      } else {
        fail("unknown field '" + st + "'");
      }
      eat(Token::kIdent);
    }

    return [getter, field, hold](const UnitreeRemote & j) {
             const KeyState & k = *getter(j);
             switch (field) {
               case Field::kPressed: return k.pressed;
               case Field::kOnPressed: return k.on_pressed;
               case Field::kOnReleased: return k.on_released;
               case Field::kHeldFor: return k.pressed && k.pressed_time >= hold;
             }
             return false;
           };
  }

  void eat(Token::Kind k)
  {
    if (tok_.kind != k) {fail("unexpected token '" + tok_.text + "'");}
    tok_ = lex_.next();
  }

  [[noreturn]] void fail(const std::string & what)
  {
    throw std::invalid_argument(what + " near position " + std::to_string(lex_.pos()));
  }

  Lexer lex_;
  Token tok_{Token::kEnd, ""};
};

}  // namespace

RemoteCondition compile_remote_condition(const std::string & expression)
{
  try {
    return Parser(expression).parse();
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument("remote condition \"" + expression + "\": " + e.what());
  } catch (const std::exception & e) {  // std::stof etc.
    throw std::invalid_argument("remote condition \"" + expression + "\": " + e.what());
  }
}

}  // namespace go2_policy_runner
