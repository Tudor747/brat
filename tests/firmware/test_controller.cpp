#include <cassert>
#include <iostream>
#include "../../config/esp_controller.ino"

std::string send(const std::string& value) {
    Serial.output.clear();
    for (char c : value + "\n") Serial.input.push_back(c);
    while (Serial.available()) loop();
    return Serial.output;
}
void expect(const std::string& cmd, const std::string& reply) {
    auto result = send(cmd);
    if (result.find(reply) == std::string::npos) {
        std::cerr << cmd << " expected " << reply << " got " << result;
        std::abort();
    }
}
int main() {
    setup();
    if (!SIMULATION && !CALIBRATION_CONFIRMED) {
        expect("ENABLE", "ERROR,UNCALIBRATED");
        expect("REF,0,0,0,0,0", "ERROR,UNCALIBRATED");
        expect("MOVE,0,0,0,0,0", "ERROR,UNCALIBRATED");
        expect("GRIP,0.5", "ERROR,UNCALIBRATED");
        assert(!enabled && !referenced);
        std::cout << "Firmware calibration lock tests passed\n";
        return 0;
    }
    expect("MOVE,0,0,0,0,0", "ERROR,DISABLED");
    expect("ENABLE", "OK,ENABLE");
    expect("MOVE,0,0,0,0,0", "ERROR,REFERENCE_REQUIRED");
    expect("REF,0,0,0,0,0", "OK,REF");
    for (auto bad : {"MOVE,nan,0,0,0,0", "MOVE,1,,0,0,0", "MOVE,1,0,0,0,0,", "GRIP,inf"})
        expect(bad, "ERROR,FORMAT");
    expect("MOVE,91,0,0,0,0", "ERROR,LIMIT");
    expect(std::string(250, 'x'), "ERROR,LINE_TOO_LONG");
    expect("PING", "OK,PING");
    expect("MOVE,5,3,-2,4,1", "OK,MOVE");
    expect("REF,0,0,0,0,0", "ERROR,BUSY");
    int ticks = 0;
    while (moving && ticks++ < 100000) {
        mockTime += 1000;
        if (ticks % 500 == 0) send("PING");
        loop();
    }
    assert(!moving && referenced && !faulted);
    long expected[5]; double target[5] = {5,3,-2,4,1};
    assert(convert(target, expected));
    for (int i = 0; i < 5; ++i) {
        assert(positionSteps[i] == expected[i]);
        if (!SIMULATION) assert(pulses[STEP_PINS[i]] == labs(expected[i]));
    }
    expect("MOVE,5,3,-2,4,1", "DONE,MOVE");
    expect("MOVE,30,3,-2,4,1", "OK,MOVE");
    mockTime += 2100000; loop();
    assert(!moving && faulted && !referenced && enabled);
    expect("CLEAR", "OK,CLEAR");
    expect("MOVE,0,0,0,0,0", "ERROR,REFERENCE_REQUIRED");
    expect("REF,0,0,0,0,0", "OK,REF");
    // Scheduler must survive micros rollover.
    mockTime = 0xfffffff0ULL;
    expect("MOVE,1,0,0,0,0", "OK,MOVE");
    for (int i = 0; moving && i < 10000; ++i) { mockTime += 1000; loop(); }
    assert(!moving && !faulted);
    if (!SIMULATION) {
        pins[LIMIT_PINS[0]] = HIGH; loop();
        assert(faulted && !referenced);
        expect("CLEAR", "ERROR,INTERLOCK");
        pins[LIMIT_PINS[0]] = LOW;
        expect("CLEAR", "OK,CLEAR");
        pins[ESTOP_PIN] = HIGH; loop();
        assert(faulted);
        pins[ESTOP_PIN] = LOW;
    }
    expect("DISABLE", "OK,DISABLE");
    assert(!enabled && !referenced && faulted);
    std::cout << "Firmware tests passed (simulation=" << SIMULATION << ")\n";
}
