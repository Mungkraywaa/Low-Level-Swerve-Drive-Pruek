#include "robot.h"

// ตารางเก็บ Pointer ของ Encoder ทุกตัวที่เปิดใช้งานอยู่
MotorEncoder *MotorEncoder::slots[MAX_DYNAMIC_ENCODERS] = {nullptr};

static IntervalTimer dynamicEncoderTimer;
static bool timerStarted = false;

// สร้าง Trampoline ISR ประจำแต่ละช่อง (0 .. 15) เพื่อให้ผูก attachInterrupt ได้โดยไม่ต้องเขียน Lambda เอง
template <uint8_t N>
static void encoderISRTrampoline()
{
    if (MotorEncoder::slots[N] != nullptr)
    {
        MotorEncoder::slots[N]->ISREncoder();
    }
}

using ISRFuncPtr = void (*)();
static const ISRFuncPtr isrTable[MAX_DYNAMIC_ENCODERS] = {
    encoderISRTrampoline<0>, encoderISRTrampoline<1>, encoderISRTrampoline<2>, encoderISRTrampoline<3>,
    encoderISRTrampoline<4>, encoderISRTrampoline<5>, encoderISRTrampoline<6>, encoderISRTrampoline<7>,
    encoderISRTrampoline<8>, encoderISRTrampoline<9>, encoderISRTrampoline<10>, encoderISRTrampoline<11>,
    encoderISRTrampoline<12>, encoderISRTrampoline<13>, encoderISRTrampoline<14>, encoderISRTrampoline<15>};

// Object มาตรฐานสำหรับ 3 ล้อ (สามารถลบออกหรือประกาศเพิ่มได้ตามต้องการ)
MotorEncoder EncoderFL, EncoderFR, EncoderB;

MotorEncoder::MotorEncoder(uint8_t encA,
                           uint8_t encB,
                           uint16_t motorPPR,
                           uint8_t maxCounter,
                           uint32_t minPulseUs,
                           bool invertDirection)
    : EncoderIN_A(encA),
      EncoderIN_B(encB),
      ppr((motorPPR > 0) ? motorPPR : 1),
      MaxCounter(maxCounter),
      MinPulseUs(minPulseUs),
      InvertDirection(invertDirection)
{
}

MotorEncoder::~MotorEncoder()
{
    detach();
}

bool MotorEncoder::init()
{
    pinMode(EncoderIN_A, INPUT_PULLUP);
    pinMode(EncoderIN_B, INPUT_PULLUP);

    // ถ้ายังไม่ได้จองช่องในตาราง slots ให้หาช่องว่างอัตโนมัติ
    if (slotIndex < 0)
    {
        for (uint8_t i = 0; i < MAX_DYNAMIC_ENCODERS; i++)
        {
            if (slots[i] == nullptr)
            {
                slotIndex = i;
                slots[i] = this;
                break;
            }
        }
    }

    if (slotIndex < 0)
        return false; // เกินจำนวน MAX_DYNAMIC_ENCODERS

    attachInterrupt(digitalPinToInterrupt(EncoderIN_A), isrTable[slotIndex], RISING);

    // เปิด IntervalTimer 1 ms อัตโนมัติเมื่อมี Encoder ตัวแรกถูก init()
    if (!timerStarted)
    {
        startEncoderTimer(1000);
    }

    return true;
}

bool MotorEncoder::init(uint8_t encA,
                        uint8_t encB,
                        uint16_t motorPPR,
                        uint8_t maxCounter,
                        uint32_t minPulseUs,
                        bool invertDirection)
{
    // ถ้าเคยผูกกับขาอื่นไว้ก่อนหน้า ให้ถอด Interrupt ขาเดิมออกก่อน
    if (slotIndex >= 0 && EncoderIN_A != encA)
    {
        detachInterrupt(digitalPinToInterrupt(EncoderIN_A));
    }

    EncoderIN_A = encA;
    EncoderIN_B = encB;
    ppr = (motorPPR > 0) ? motorPPR : 1;
    MaxCounter = maxCounter;
    MinPulseUs = minPulseUs;
    InvertDirection = invertDirection;

    return init();
}

void MotorEncoder::detach()
{
    if (slotIndex >= 0 && slotIndex < MAX_DYNAMIC_ENCODERS)
    {
        detachInterrupt(digitalPinToInterrupt(EncoderIN_A));
        slots[slotIndex] = nullptr;
        slotIndex = -1;
    }
}

void MotorEncoder::setConfig(uint16_t motorPPR, uint8_t maxCounter, uint32_t minPulseUs, bool invertDirection)
{
    ppr = (motorPPR > 0) ? motorPPR : 1;
    MaxCounter = maxCounter;
    MinPulseUs = minPulseUs;
    InvertDirection = invertDirection;
}

void MotorEncoder::setPPR(uint16_t motorPPR)
{
    ppr = (motorPPR > 0) ? motorPPR : 1;
}

void MotorEncoder::setMaxCounter(uint8_t maxCounter)
{
    MaxCounter = maxCounter;
}

void MotorEncoder::setMinPulseUs(uint32_t minPulseUs)
{
    MinPulseUs = minPulseUs;
}

void MotorEncoder::setInvertDirection(bool invert)
{
    InvertDirection = invert;
}

void MotorEncoder::setPin(uint8_t encA, uint8_t encB, uint16_t motorPPR)
{
    init(encA, encB, motorPPR, MaxCounter, MinPulseUs, InvertDirection);
}

void MotorEncoder::ISREncoder()
{
    IsrCount++;

    uint32_t CurrentTime = micros();
    uint32_t dt = CurrentTime - PrevTime;

    // กรองสัญญาณรบกวนด้วยค่า MinPulseUs ที่กำหนดไว้ในแต่ละ Encoder
    if (dt < MinPulseUs)
    {
        FilteredCount++;
        return;
    }

    DeltaTime = dt;
    PrevTime = CurrentTime;

    bool bLow = (digitalReadFast(EncoderIN_B) == LOW);
    bool dirPos = InvertDirection ? !bLow : bLow;

    if (dirPos)
    {
        MotorDirection = 1;
        EncoderCount++;
    }
    else
    {
        MotorDirection = -1;
        EncoderCount--;
    }
}

static float wrapAngleDeg(float deg)
{
    float wrapped = fmodf(deg + 180.0f, 360.0f);
    if (wrapped < 0.0f)
        wrapped += 360.0f;
    return wrapped - 180.0f;
}

void MotorEncoder::ReadRPM()
{
    int32_t countSnapshot = EncoderCount;
    uint32_t dtSnapshot = DeltaTime;
    int8_t dirSnapshot = MotorDirection;

    if (countSnapshot == PrevEncoderCount)
    {
        if (Counter < MaxCounter)
            Counter++;
    }
    else
    {
        Counter = 0;
    }
    PrevEncoderCount = countSnapshot;

    if (Counter < MaxCounter && dtSnapshot > 0)
    {
        CurrentRPM = (int16_t)((60000000UL / (dtSnapshot * ppr)) * dirSnapshot);
    }
    else
    {
        CurrentRPM = 0;
        MotorDirection = 0;
    }

    // คำนวณมุมจากจำนวน Pulse หลังหัก Offset
    int32_t netCount = countSnapshot - PulseOffset;
    float total_deg = (netCount * 360.0f) / (float)ppr;
    TotalAngle = total_deg;
    CurrentAngle = wrapAngleDeg(total_deg);
}

void MotorEncoder::setOffset(int32_t target_pulse)
{
    noInterrupts();
    int32_t raw = EncoderCount;
    interrupts();

    PulseOffset = raw - target_pulse;

    // อัปเดตมุมทันทีหลังเซ็ต Offset
    float total_deg = (target_pulse * 360.0f) / (float)ppr;
    TotalAngle = total_deg;
    CurrentAngle = wrapAngleDeg(total_deg);
}

void MotorEncoder::reset()
{
    noInterrupts();
    EncoderCount = 0;
    PulseOffset = 0;
    DeltaTime = 0;
    MotorDirection = 0;
    IsrCount = 0;
    FilteredCount = 0;
    interrupts();

    PrevEncoderCount = 0;
    Counter = MaxCounter;
    CurrentRPM = 0;
    CurrentAngle = 0.0f;
    TotalAngle = 0.0f;
}

void MotorEncoder::debug(const char *name, uint32_t interval_ms)
{
    if (debugtime < interval_ms)
        return;
    debugtime = 0;

    char prefix[24];
    if (name != nullptr && name[0] != '\0')
    {
        snprintf(prefix, sizeof(prefix), "%s", name);
    }
    else
    {
        snprintf(prefix, sizeof(prefix), "Enc%u", EncoderIN_A);
    }

    Serial.printf(">%s_Slot: %d\n", prefix, slotIndex);
    Serial.printf(">%s_TimerOn: %d\n", prefix, timerStarted ? 1 : 0);
    Serial.printf(">%s_PPR: %u\n", prefix, ppr);
    Serial.printf(">%s_PinA: %d\n", prefix, digitalRead(EncoderIN_A));
    Serial.printf(">%s_PinB: %d\n", prefix, digitalRead(EncoderIN_B));
    Serial.printf(">%s_IsrHits: %lu\n", prefix, (unsigned long)IsrCount);
    Serial.printf(">%s_Filtered: %lu\n", prefix, (unsigned long)FilteredCount);
    Serial.printf(">%s_Pulse: %ld\n", prefix, (long)getCount());
    Serial.printf(">%s_RawPulse: %ld\n", prefix, (long)getRawCount());
    Serial.printf(">%s_Offset: %ld\n", prefix, (long)PulseOffset);
    Serial.printf(">%s_Dir: %d\n", prefix, (int)MotorDirection);
    Serial.printf(">%s_DeltaUs: %lu\n", prefix, (unsigned long)DeltaTime);
    Serial.printf(">%s_IdleCnt: %u\n", prefix, Counter);
    Serial.printf(">%s_RPM: %d\n", prefix, getRPM());
    Serial.printf(">%s_Angle: %.2f\n", prefix, getAngle());
    Serial.printf(">%s_TotalAngle: %.2f\n\n", prefix, getTotalAngle());
}

void updateAllRPM()
{
    for (uint8_t i = 0; i < MAX_DYNAMIC_ENCODERS; i++)
    {
        if (MotorEncoder::slots[i] != nullptr)
        {
            MotorEncoder::slots[i]->ReadRPM();
        }
    }
}

void startEncoderTimer(uint32_t interval_us)
{
    dynamicEncoderTimer.begin(updateAllRPM, interval_us);
    timerStarted = true;
}
