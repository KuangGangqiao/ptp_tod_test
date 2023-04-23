#include <stdio.h>
#include <time.h>

int main() {
    time_t current_time = 1634343442;
    struct tm* utc_time = gmtime(&current_time);
    char time_string[80];
    strftime(time_string, sizeof(time_string), "%Y-%m-%d %H:%M:%S", utc_time);
    printf("Current time: %s\n", time_string);
    return 0;
}
