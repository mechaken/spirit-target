/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <cfloat>
#include <cmath>

#include "A3921.h"
#include "MdIdDipSW.h"
#include "MdLed.h"
#include "Motor.h"
#include "SMPTarget.h"
#include "SpiritCANTarget.h"
#include "mbed.h"

using namespace spirit;

void coordinate(const Motor& target, Motor& actual);

static constexpr int  pc_baud   = 115'200;
static constexpr auto loop_rate = 1ms;

int main()
{
    BufferedSerial pc(USBTX, USBRX, pc_baud);

    MdIdDipSW dip_sw(PB_3, PB_4, PB_5, PB_6);  //下位ビット ~ 上位ビット
    CAN       can(PA_11, PA_12);               // RD, TD

    can.filter(dip_sw.id(), 0x7FFU);

    SMPTarget       smp;
    SpiritCANTarget spirit_can(can);

    printf("Spirit-Target v0.0.1\n");
    printf("Spirit Motor Protocol(SMP) v0.0.1\n");
    printf("Spirit CAN v0.0.1\n");

    Motor target;
    Motor actual;
    A3921 a3921(PB_0, PA_6, PA_7, PB_1, PA_5);
    MdLed led(PA_10, PA_9);

    led = MdLed::Mode::Alternate;

    printf("Motor Base ID is %d. (DIP SW = %x)\n", dip_sw.id(), dip_sw.read());

    while (true) {
        // データ受信判定
        if (spirit_can.read()) {
            // データの再構築処理の完了判定
            if (spirit_can.has_new_message()) {
                smp.decode(spirit_can.get_smp_data(), target);
                led = target.state();
                debug("%d, %3.2f\r", static_cast<int>(target.state()), target.duty_cycle());
            }
        }

        coordinate(target, actual);
        led.coordinate();

        a3921.set(actual.duty_cycle(), actual.state());

        ThisThread::sleep_for(loop_rate);
    }
}

// change level に応じてデューティー比の増減具合を調整する
void coordinate(const Motor& target, Motor& actual)
{
    if (actual.ttl() == 0) {
        actual = 0.00F;
        actual = Motor::default_state;

        return;
    }

    actual.ttl(actual.ttl() - 1);

    // 回転方向が異なる場合
    if (actual.state() != target.state()) {
        if (target.fall_level() == ChangeLevel::OFF) {
            actual = target.duty_cycle();
            actual = target.state();
        } else if (actual.duty_cycle() <= target.fall_unit()) {
            actual = 0.00F;
            actual = target.state();
        } else {
            actual -= target.fall_unit();
        }

        return;
    }
    // デューティー比が異なる場合
    else if (fabs(target.duty_cycle() - actual.duty_cycle()) > FLT_EPSILON) {
        if (target.state() == State::Brake || target.state() == State::Free) {
            return;
        }

        // 設定値の方が現在のデューティー比より大きい場合
        if ((target.duty_cycle() - actual.duty_cycle()) >= target.rise_unit()) {
            if (target.rise_level() == ChangeLevel::OFF) {
                actual = target.duty_cycle();
            } else {
                actual += target.rise_unit();
            }
        }
        // 設定値の方が現在のデューティー比より小さい場合
        else if ((target.duty_cycle() - actual.duty_cycle()) <= -target.fall_unit()) {
            if (target.fall_level() == ChangeLevel::OFF) {
                actual = target.duty_cycle();
            } else {
                actual -= target.fall_unit();
            }
        }
        // 設定値と現在のデューティー比がほぼ一緒の時
        {
            actual = target.duty_cycle();
        }

        return;
    }

    return;
}
