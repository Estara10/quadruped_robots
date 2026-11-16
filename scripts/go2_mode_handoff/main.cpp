#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
void report(const char* phase, int code, const std::string& form, const std::string& mode) {
    std::cout << std::fixed << std::setprecision(9)
              << "phase=" << phase << " steady_s="
              << std::chrono::duration<double>(Clock::now().time_since_epoch()).count()
              << " return=" << code << " form=" << std::quoted(form)
              << " mode=" << std::quoted(mode) << std::endl;
}
}

int main(int argc, char** argv) {
    // Actual 2026-11-04 test: SelectMode("ai") caused an unintended stand.
    // Freeze this diagnostic; the executed source is retained with the run logs.
    std::cerr << "DISABLED: restoring native ai caused an unintended stand on this Go2.\n"
              << "Do not repeat this prone handoff. Review recovery/support before any new control test.\n";
    return 2;

    if (argc != 5 || std::string(argv[1]) != "--domain" ||
        std::string(argv[3]) != "--interface") {
        std::cerr << "CHANGES NATIVE CONTROL MODE; supported, prone Go2 EDU only.\n"
                  << "Usage: " << argv[0] << " --domain 0 --interface <confirmed NIC>\n";
        return 2;
    }
    // This is the bounded check for this known Go2 connection, not a generic launcher.
    if (std::string(argv[2]) != "0" || std::string(argv[4]) == "lo" || !argv[4][0]) {
        std::cerr << "Requires confirmed real interface and domain 0\n";
        return 2;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[4]);
    } catch (const std::exception& error) {
        std::cerr << "DDS initialization failed: " << error.what() << '\n';
        return 1;
    }
    int result = 0;
    {
        unitree::robot::b2::MotionSwitcherClient client;
        std::string original_form, original_mode;
        bool restore_needed = false;
        try {
            client.SetTimeout(2.0f);
            client.Init();
            int code = client.CheckMode(original_form, original_mode);
            report("baseline_mode", code, original_form, original_mode);
            if (code != 0 || original_form != "0" || original_mode != "ai")
                throw std::runtime_error("Expected confirmed Go2 ai mode; no release performed");
            if (interrupted) throw std::runtime_error("Interrupted before release");

            // Preserve restoration even when a request times out after taking effect.
            restore_needed = true;
            report("release_request", 0, original_form, original_mode);
            code = client.ReleaseMode();
            report("release_return", code, "", "");
            if (code != 0) result = 1;
            bool released = false;
            const auto deadline = Clock::now() + std::chrono::seconds(3);
            do {
                std::string form, mode;
                code = client.CheckMode(form, mode);
                report("release_query", code, form, mode);
                if (code == 0 && mode.empty()) {
                    released = true;
                    break;
                }
                if (interrupted) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            } while (Clock::now() < deadline);
            if (!released) throw std::runtime_error("Native release NOT confirmed");
            report("released_observe_begin", 0, "", "");
            for (int i = 0; i < 20 && !interrupted; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            report("released_observe_end", 0, "", "");
            if (interrupted) result = 1;
        } catch (const std::exception& error) {
            result = 1;
            std::cerr << "Handoff check failed: " << error.what() << '\n';
        }

        // Cleanup deliberately runs even after SIGINT/SIGTERM. It cannot cover SIGKILL/power loss.
        if (restore_needed) {
            bool restored = false;
            bool select_pending = false;
            try {
                for (int attempt = 0; attempt < 5; ++attempt) {
                    std::string form, mode;
                    int code = client.CheckMode(form, mode);
                    report("restore_query", code, form, mode);
                    if (code == 0 && form == original_form && mode == original_mode) {
                        restored = true;
                        break;
                    }
                    if (code == 0 && !mode.empty()) {
                        std::cerr << "Unexpected active native mode; not overwriting it\n";
                        break;
                    }
                    if (!select_pending) {
                        report("restore_request", 0, original_form, original_mode);
                        code = client.SelectMode(original_mode);
                        report("restore_return", code, original_form, original_mode);
                        if (code == 0) select_pending = true;
                        // SDK documents 7002 as BUSY; retry only this known transient.
                        else if (code != 7002) {
                            result = 1;
                            break;
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                if (!restored) {
                    std::string form, mode;
                    const int code = client.CheckMode(form, mode);
                    report("restore_final_query", code, form, mode);
                    restored = code == 0 && form == original_form && mode == original_mode;
                }
            } catch (const std::exception& error) {
                std::cerr << "Restoration exception: " << error.what() << '\n';
            }
            if (restored) report("restore_confirmed", 0, original_form, original_mode);
            else {
                result = 3;
                report("restore_NOT_confirmed", result, original_form, original_mode);
                std::cerr << "KEEP ROBOT SUPPORTED. Native/remote control is NOT confirmed. "
                             "Do not stand or move; manual recovery is required.\n";
            }
        }
    }
    unitree::robot::ChannelFactory::Instance()->Release();
    return result;
}
