#define SIMPLEFIFO_LARGE
#include <SimpleFIFO.h>

#define R_PIN 6
#define G_PIN 5
#define B_PIN 3
#define WARN_FULL_PIN 13
#define WARN_EMPTY_PIN 12
#define WARN_EMPTY_GND_PIN 11
#define BUFFER_LENGTH (512U)
#define LED_UPDATE_PERIOD_MS (2)

SimpleFIFO<uint8_t, BUFFER_LENGTH> buffer_r;
SimpleFIFO<uint8_t, BUFFER_LENGTH> buffer_g;
SimpleFIFO<uint8_t, BUFFER_LENGTH> buffer_b;
uint32_t last_update_ms = 0;
uint32_t time_ms = 0;
uint8_t delay_time_ms = LED_UPDATE_PERIOD_MS;

void setup()
{
    pinMode(R_PIN, OUTPUT);
    pinMode(G_PIN, OUTPUT);
    pinMode(B_PIN, OUTPUT);
    pinMode(WARN_EMPTY_PIN, OUTPUT);
    pinMode(WARN_FULL_PIN, OUTPUT);
    pinMode(WARN_EMPTY_GND_PIN, OUTPUT);
    Serial.begin(250000);
}

void loop()
{
    bool buffer_full = buffer_r.count()>=(BUFFER_LENGTH-8)||buffer_g.count()>=(BUFFER_LENGTH-8)||buffer_b.count()>=(BUFFER_LENGTH-8);
    bool buffer_empty = buffer_r.count()<=3||buffer_g.count()<=3||buffer_b.count()<=3;

    digitalWrite(WARN_FULL_PIN, buffer_full);
    digitalWrite(WARN_EMPTY_PIN, buffer_empty);

    if (buffer_full && delay_time_ms > 0) delay_time_ms--;
    else if (buffer_empty && delay_time_ms < LED_UPDATE_PERIOD_MS + 4) delay_time_ms++;

    time_ms = millis();
    
    if (time_ms > last_update_ms + delay_time_ms)
    {
        updateLEDs();
        last_update_ms = time_ms;
    }
    else if (time_ms < last_update_ms)
    {
        /* Millis() overflow handling */
        last_update_ms = 0;
    }
}

void updateLEDs()
{
    if (buffer_r.count()>0) analogWrite(R_PIN, buffer_r.dequeue());
    if (buffer_g.count()>0) analogWrite(G_PIN, buffer_g.dequeue());
    if (buffer_b.count()>0) analogWrite(B_PIN, buffer_b.dequeue());
}

void serialEvent()
{
    while (Serial.available())
    {
        char in = (char)Serial.read();
        switch (in)
        {
        case 'r':
            buffer_r.enqueue((uint8_t)Serial.parseInt());
            break;
        case 'g':
            buffer_g.enqueue((uint8_t)Serial.parseInt());
            break;
        case 'b':
            buffer_b.enqueue((uint8_t)Serial.parseInt());
            break;
        default:
            break;
        }
    }
}
