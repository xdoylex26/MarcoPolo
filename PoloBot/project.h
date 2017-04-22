#include <Arduino.h>
#define VER "PoloBot"
#define AUTHORS "Branden, Mark, Nick"


void prelude_report(void);
inline void prelude_report(void)
{
    Serial.print("EN.525.410 Final Design\n");
    Serial.print("Version: "); Serial.println(VER);
    Serial.print(__DATE__); Serial.print(", "); Serial.println(__TIME__);
    Serial.print("Authors: "); Serial.println(AUTHORS); Serial.println("");
}
