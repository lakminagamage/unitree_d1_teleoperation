/**
 * D1 Arm Joint Controller - Interactive TUI
 * 
 * Build alongside your other D1 SDK examples by adding to CMakeLists.txt:
 *   add_executable(d1_joint_controller d1_joint_controller.cpp src/msg/ArmString_.cpp)
 *   target_link_libraries(d1_joint_controller unitree_sdk2 dds dcps rt pthread ncurses)
 *
 * Run: ./d1_joint_controller
 *
 * Controls:
 *   UP/DOWN arrows  - select joint
 *   LEFT/RIGHT arrows - decrease/increase angle by 1° (or 1mm for gripper)
 *   [ / ]           - decrease/increase angle by 10° (or 5mm for gripper)
 *   o               - open gripper fully (65mm)
 *   c               - close gripper fully (0mm)
 *   SPACE           - send all joints to robot (all-joint mode)
 *   z               - zero all joints (return to zero)
 *   e               - enable all motors
 *   d               - disable (release) all motors
 *   q               - quit
 */

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"

#include <ncurses.h>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>

using namespace unitree::robot;
using namespace unitree::common;

#define TOPIC "rt/arm_Command"

// Joint configuration
struct JointInfo {
    const char* name;
    float min_deg;
    float max_deg;
    float value;       // current target angle
};

static JointInfo joints[] = {
    { "J0 (Base Rotation)",  -135.0f,  135.0f, 0.0f },
    { "J1 (Shoulder)",        -90.0f,   90.0f, 0.0f },
    { "J2 (Elbow)",           -90.0f,   90.0f, 0.0f },
    { "J3 (Forearm Roll)",   -135.0f,  135.0f, 0.0f },
    { "J4 (Wrist Pitch)",     -90.0f,   90.0f, 0.0f },
    { "J5 (Wrist Roll)",     -135.0f,  135.0f, 0.0f },
    { "J6 (Claw 0-65mm)",      0.0f,   65.0f, 0.0f },
};
static const int NUM_JOINTS = 7;

// Clamp value within joint limits
float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Build JSON for all-joint angle control (funcode 2, mode 1 = smooth trajectory)
std::string buildAllJointCmd(int seq) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{\"seq\":" << seq
        << ",\"address\":1"
        << ",\"funcode\":2"
        << ",\"data\":{"
        << "\"mode\":1"
        << ",\"angle0\":" << joints[0].value
        << ",\"angle1\":" << joints[1].value
        << ",\"angle2\":" << joints[2].value
        << ",\"angle3\":" << joints[3].value
        << ",\"angle4\":" << joints[4].value
        << ",\"angle5\":" << joints[5].value
        << ",\"angle6\":" << joints[6].value
        << "}}";
    return oss.str();
}

// Build JSON for single joint control (funcode 1)
std::string buildSingleJointCmd(int seq, int joint_id, float angle) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "{\"seq\":" << seq
        << ",\"address\":1"
        << ",\"funcode\":1"
        << ",\"data\":{"
        << "\"id\":" << joint_id
        << ",\"angle\":" << angle
        << ",\"delay_ms\":0"
        << "}}";
    return oss.str();
}

// Enable all motors (funcode 5, mode 1)
std::string buildEnableCmd(int seq) {
    std::ostringstream oss;
    oss << "{\"seq\":" << seq << ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":1}}";
    return oss.str();
}

// Disable/release all motors (funcode 5, mode 0)
std::string buildDisableCmd(int seq) {
    std::ostringstream oss;
    oss << "{\"seq\":" << seq << ",\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
    return oss.str();
}

// Return to zero (funcode 7)
std::string buildZeroCmd(int seq) {
    std::ostringstream oss;
    oss << "{\"seq\":" << seq << ",\"address\":1,\"funcode\":7}";
    return oss.str();
}

void drawUI(int selected, const std::string& lastCmd, const std::string& status) {
    clear();
    int row = 0;

    // Header
    attron(A_BOLD | COLOR_PAIR(1));
    mvprintw(row++, 0, "+=========================================================+");
    mvprintw(row++, 0, "|        Unitree D1 Arm - Joint Controller                |");
    mvprintw(row++, 0, "+=========================================================+");
    attroff(A_BOLD | COLOR_PAIR(1));
    row++;

    // Column headers
    attron(A_BOLD);
    mvprintw(row++, 2, "%-22s  %7s  %7s  %7s  %s", "Joint", "Min", "Target", "Max", "Slider");
    attroff(A_BOLD);
    mvprintw(row++, 2, "---------------------------------------------------------");

    // Joint rows
    for (int i = 0; i < NUM_JOINTS; i++) {
        const JointInfo& j = joints[i];
        bool isSel = (i == selected);

        // Highlight selected row
        if (isSel) attron(A_REVERSE | COLOR_PAIR(2));

        float range = j.max_deg - j.min_deg;
        float pct   = (j.value - j.min_deg) / range;  // 0..1
        int   barW  = 20;
        int   filled = (int)(pct * barW + 0.5f);
        filled = filled < 0 ? 0 : (filled > barW ? barW : filled);

        // Bar (plain ASCII)
        char bar[32];
        for (int b = 0; b < barW; b++) bar[b] = (b < filled) ? '#' : '-';
        bar[barW] = '\0';

        const char* unit = (i == 6) ? "mm" : "dg";
        mvprintw(row, 2, " %-22s %5.1f%s  %5.1f%s  %5.1f%s  [%s]",
            j.name,
            j.min_deg, unit,
            j.value,   unit,
            j.max_deg, unit,
            bar);

        if (isSel) attroff(A_REVERSE | COLOR_PAIR(2));
        row++;
    }

    row++;
    mvprintw(row++, 2, "---------------------------------------------------------");

    // Status line
    attron(COLOR_PAIR(3));
    mvprintw(row++, 2, " Status: %-50s", status.c_str());
    attroff(COLOR_PAIR(3));

    // Last command
    attron(COLOR_PAIR(4));
    mvprintw(row++, 2, " Cmd:    %-50s", lastCmd.substr(0, 55).c_str());
    attroff(COLOR_PAIR(4));

    row++;

    // Controls help
    attron(A_DIM);
    mvprintw(row++, 2, "  UP/DN  Select joint    LT/RT  Angle +/-1deg/mm    [ ]  Angle +/-10deg/5mm");
    mvprintw(row++, 2, "  o  Gripper open    c  Gripper close    SPACE  Send all joints");
    mvprintw(row++, 2, "  z  Zero all    e  Enable    d  Disable    q  Quit");
    attroff(A_DIM);

    refresh();
}

int main(int argc, char** argv) {
    // Accept optional network interface as first argument, default to known interface
    std::string netif = "enx00e04c680099";
    if (argc > 1) netif = argv[1];

    // Init DDS publisher with network interface
    ChannelFactory::Instance()->Init(0, netif.c_str());
    ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(TOPIC);
    publisher.InitChannel();

    // Give DDS time to establish before ncurses takes over stdout
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Init ncurses
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    nodelay(stdscr, FALSE);  // blocking input
    set_escdelay(25);

    if (has_colors()) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_CYAN,    -1);  // header
        init_pair(2, COLOR_BLACK,   COLOR_GREEN);  // selected row
        init_pair(3, COLOR_GREEN,   -1);  // status
        init_pair(4, COLOR_YELLOW,  -1);  // last cmd
    }

    int selected  = 0;
    int seq       = 1;
    std::string lastCmd = "(none)";
    std::string status  = "Ready. Connect arm and press 'e' to enable motors.";

    unitree_arm::msg::dds_::ArmString_ msg{};

    drawUI(selected, lastCmd, status);

    while (true) {
        int ch = getch();

        if (ch == 'q' || ch == 'Q') {
            break;
        }

        bool needRedraw = true;

        switch (ch) {
            case KEY_UP:
                selected = (selected - 1 + NUM_JOINTS) % NUM_JOINTS;
                status = "Selected: " + std::string(joints[selected].name);
                break;

            case KEY_DOWN:
                selected = (selected + 1) % NUM_JOINTS;
                status = "Selected: " + std::string(joints[selected].name);
                break;

            case KEY_LEFT: {
                float step = 1.0f;
                joints[selected].value = clamp(joints[selected].value - step,
                                               joints[selected].min_deg,
                                               joints[selected].max_deg);
                // Send single joint command immediately
                lastCmd = buildSingleJointCmd(seq, selected, joints[selected].value);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                std::string unit = (selected == 6) ? "mm" : "deg";
                status = "Sent J" + std::to_string(selected) + " = " +
                         std::to_string((int)joints[selected].value) + unit;
                break;
            }

            case KEY_RIGHT: {
                float step = 1.0f;
                joints[selected].value = clamp(joints[selected].value + step,
                                               joints[selected].min_deg,
                                               joints[selected].max_deg);
                lastCmd = buildSingleJointCmd(seq, selected, joints[selected].value);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                std::string unit = (selected == 6) ? "mm" : "deg";
                status = "Sent J" + std::to_string(selected) + " = " +
                         std::to_string((int)joints[selected].value) + unit;
                break;
            }

            case '[': {
                float step = (selected == 6) ? 5.0f : 10.0f;
                joints[selected].value = clamp(joints[selected].value - step,
                                               joints[selected].min_deg,
                                               joints[selected].max_deg);
                lastCmd = buildSingleJointCmd(seq, selected, joints[selected].value);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                std::string unit = (selected == 6) ? "mm" : "deg";
                status = "Sent J" + std::to_string(selected) + " = " +
                         std::to_string((int)joints[selected].value) + unit;
                break;
            }

            case ']': {
                float step = (selected == 6) ? 5.0f : 10.0f;
                joints[selected].value = clamp(joints[selected].value + step,
                                               joints[selected].min_deg,
                                               joints[selected].max_deg);
                lastCmd = buildSingleJointCmd(seq, selected, joints[selected].value);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                std::string unit = (selected == 6) ? "mm" : "deg";
                status = "Sent J" + std::to_string(selected) + " = " +
                         std::to_string((int)joints[selected].value) + unit;
                break;
            }

            case 'o': case 'O': {
                // Open gripper fully
                joints[6].value = 65.0f;
                lastCmd = buildSingleJointCmd(seq, 6, 65.0f);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                status = "Gripper OPEN (65mm)";
                break;
            }

            case 'c': case 'C': {
                // Close gripper fully
                joints[6].value = 0.0f;
                lastCmd = buildSingleJointCmd(seq, 6, 0.0f);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                status = "Gripper CLOSED (0mm)";
                break;
            }

            case ' ': {
                // Send all joints at once
                lastCmd = buildAllJointCmd(seq);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                status = "Sent ALL joints (smooth trajectory mode)";
                break;
            }

            case 'z': case 'Z': {
                // Reset local values to zero then send zero command
                for (int i = 0; i < NUM_JOINTS; i++) joints[i].value = 0.0f;
                lastCmd = buildZeroCmd(seq);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                status = "Zero command sent - arm returning to zero position";
                break;
            }

            case 'e': case 'E': {
                lastCmd = buildEnableCmd(seq);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                status = "Motors ENABLED";
                break;
            }

            case 'd': case 'D': {
                lastCmd = buildDisableCmd(seq);
                msg.data_() = lastCmd;
                publisher.Write(msg);
                seq++;
                status = "Motors DISABLED (released)";
                break;
            }

            default:
                needRedraw = false;
                break;
        }

        if (needRedraw) {
            drawUI(selected, lastCmd, status);
        }
    }

    endwin();
    return 0;
}