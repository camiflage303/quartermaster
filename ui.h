#pragma once
#include <Arduino.h>

namespace ui {

    /* call once from setup() */
    void init();

    /* call every loop() – cheap; only touches the strip when something changed */
    void refresh();
}
