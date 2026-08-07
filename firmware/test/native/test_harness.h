// Minimal test harness for the Jensen Hand control core.
//
// Deliberately dependency-free: the point of `lib/jensen_core` is that it
// builds anywhere with a C++17 compiler, and a test runner that needs its own
// package manager would undo that. Registration is by static constructor, so a
// test is added by writing `TEST(name) { ... }` and nothing else.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

namespace jtest {

struct Failure {
    const char* file;
    int line;
    char message[512];
};

class Registry {
public:
    using Fn = void (*)();

    struct Entry {
        const char* name;
        Fn fn;
    };

    static Registry& instance() {
        static Registry registry;
        return registry;
    }

    void add(const char* name, Fn fn) { entries_.push_back({name, fn}); }

    void fail(const char* file, int line, const char* message) {
        Failure failure{};
        failure.file = file;
        failure.line = line;
        std::snprintf(failure.message, sizeof(failure.message), "%s", message);
        failures_.push_back(failure);
    }

    int run() {
        int failed = 0;
        for (const Entry& entry : entries_) {
            failures_.clear();
            entry.fn();
            if (failures_.empty()) {
                std::printf("  ok    %s\n", entry.name);
            } else {
                ++failed;
                std::printf("  FAIL  %s\n", entry.name);
                for (const Failure& failure : failures_) {
                    std::printf("        %s:%d: %s\n", failure.file, failure.line, failure.message);
                }
            }
        }
        std::printf("\n%d test(s), %d failed\n", static_cast<int>(entries_.size()), failed);
        return failed == 0 ? 0 : 1;
    }

private:
    std::vector<Entry> entries_;
    std::vector<Failure> failures_;
};

struct Registrar {
    Registrar(const char* name, Registry::Fn fn) { Registry::instance().add(name, fn); }
};

inline bool nearlyEqual(double a, double b, double tolerance) {
    return std::fabs(a - b) <= tolerance;
}

}  // namespace jtest

#define TEST(name)                                                     \
    static void name();                                                \
    static ::jtest::Registrar name##_registrar(#name, &name);          \
    static void name()

#define JT_FAIL(msg) ::jtest::Registry::instance().fail(__FILE__, __LINE__, msg)

#define CHECK(expr)                                    \
    do {                                               \
        if (!(expr)) JT_FAIL("CHECK failed: " #expr);  \
    } while (0)

#define CHECK_FALSE(expr)                                    \
    do {                                                     \
        if ((expr)) JT_FAIL("CHECK_FALSE failed: " #expr);   \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                                                 \
    do {                                                                                        \
        const double jt_a = static_cast<double>(actual);                                        \
        const double jt_e = static_cast<double>(expected);                                      \
        if (!::jtest::nearlyEqual(jt_a, jt_e, static_cast<double>(tolerance))) {                \
            char jt_buf[512];                                                                   \
            std::snprintf(jt_buf, sizeof(jt_buf),                                               \
                          "CHECK_NEAR failed: " #actual " = %.6f, expected %.6f (tol %.6f)",    \
                          jt_a, jt_e, static_cast<double>(tolerance));                          \
            JT_FAIL(jt_buf);                                                                    \
        }                                                                                       \
    } while (0)

#define CHECK_EQ(actual, expected)                                                    \
    do {                                                                              \
        const long long jt_a = static_cast<long long>(actual);                        \
        const long long jt_e = static_cast<long long>(expected);                      \
        if (jt_a != jt_e) {                                                           \
            char jt_buf[512];                                                         \
            std::snprintf(jt_buf, sizeof(jt_buf),                                     \
                          "CHECK_EQ failed: " #actual " = %lld, expected %lld",       \
                          jt_a, jt_e);                                                \
            JT_FAIL(jt_buf);                                                          \
        }                                                                             \
    } while (0)

#define CHECK_GT(a, b)                                          \
    do {                                                        \
        if (!((a) > (b))) JT_FAIL("CHECK_GT failed: " #a " > " #b); \
    } while (0)

#define CHECK_LT(a, b)                                          \
    do {                                                        \
        if (!((a) < (b))) JT_FAIL("CHECK_LT failed: " #a " < " #b); \
    } while (0)
