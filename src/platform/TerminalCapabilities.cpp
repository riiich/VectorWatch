#include "vectorwatch/platform/TerminalCapabilities.hpp"

#include <cstdio>

#ifdef _WIN32
#include <io.h>
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace vectorwatch {

bool terminalSupportsInPlaceRendering() noexcept {
#ifdef _WIN32
    if (_isatty(_fileno(stdout)) == 0) {
        return false;
    }
    const HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
    if (output == INVALID_HANDLE_VALUE || output == nullptr) {
        return false;
    }
    DWORD mode = 0;
    if (GetConsoleMode(output, &mode) == 0) {
        return false;
    }
    return SetConsoleMode(
               output,
               mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING) != 0;
#else
    return isatty(STDOUT_FILENO) != 0;
#endif
}

} // namespace vectorwatch
