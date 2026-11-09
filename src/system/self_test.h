#pragma once

class AppController;

/**
 * @brief 9-stage visual display self-test demo sequence and mathematical assertions.
 */
class SelfTest {
public:
    static void run(AppController &app);
    static int runAssertions(int *failedTests, int maxFails);
};
