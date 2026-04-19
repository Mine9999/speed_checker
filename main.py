import machine
import time
import framebuf
from ssd1306 import SSD1306_SPI

# --- 設定 ---
ROLLER_DIAMETER = 20.0
CIRCUMFERENCE = ROLLER_DIAMETER * 3.14159 / 1000.0
MIN_PULSE_INTERVAL_US = 1200  # デバウンス時間1.2ms(約50km/h以上に対応するため、1.2msに設定)
MAX_1REV_US = 5000000  # 1周あたり最大5秒まで許容（低速手動確認用）
STOP_TIMEOUT_US = 1000000  # 最後のパルスから1秒で停止判定
MAX_UPDATE_DELAY_MS = 3000  # MAX値更新開始までの遅延時間（3秒）
MAX_RESET_AFTER_STOP_MS = 5000  # 停止後5秒経過で次回リセット判定
STOP_SPEED_THRESHOLD = 3.0  # この速度以下を停止判定とみなす(km/h)
DISPLAY_SPEED_RESET_THRESHOLD = 5.0  # 停止後リセットを発動する最低速度(km/h)


# --- ピン設定 ---
spi = machine.SPI(0, baudrate=1000000, sck=machine.Pin(18), mosi=machine.Pin(19))
res = machine.Pin(20)
dc  = machine.Pin(21)
cs  = machine.Pin(17)
oled = SSD1306_SPI(128, 32, spi, dc, res, cs)

def big_text(text, x, y):
    for char in text:
        char_buf = bytearray(8)
        char_fb = framebuf.FrameBuffer(char_buf, 8, 8, framebuf.MONO_VLSB)
        char_fb.text(char, 0, 0, 1)
        for i in range(8):
            for j in range(8):
                if (char_buf[i] >> j) & 0x01:
                    oled.fill_rect(x + i*2, y + j*2, 2, 2, 1)
        x += 16

sensor = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP)

# --- 割り込み(IRQ)専用の超軽量変数 ---
time_buf = [0, 0, 0]
t_idx = 0
valid_duration = 0
new_data_flag = False
last_pulse_time = 0
last_step_time = 0  # ★追加：直近の磁石間の時間

# --- メインループ用の変数 ---
history = []
display_speed = 0.0
max_speed = 0.0
start_measuring_time = 0
stop_time_ms = 0
reset_max_on_next_speed = False
# 【パルス抜け検知付きの割り込み処理】
def calculate_speed(pin):
    global t_idx, valid_duration, new_data_flag, last_pulse_time, last_step_time
    now = time.ticks_us()
    
    if last_pulse_time > 0:
        step_time = time.ticks_diff(now, last_pulse_time)
        
        # 1. チャタリング（ノイズ）弾き
        if step_time < MIN_PULSE_INTERVAL_US:
            return
            
        # 2. パルス抜け検知（前回の磁石間隔の1.5倍以上の時間がかかったら無視）
        if last_step_time > 0 and step_time > (last_step_time * 1.5):
            # 異常データが混ざるのでバッファをリセットして測り直す
            time_buf[0] = 0
            time_buf[1] = 0
            time_buf[2] = 0
            last_step_time = step_time
            last_pulse_time = now
            return
            
        last_step_time = step_time
        
    # 通常の1周計算
    oldest_time = time_buf[t_idx]
    time_buf[t_idx] = now
    last_pulse_time = now
    
    if oldest_time > 0:
        duration_1rev = time.ticks_diff(now, oldest_time)
        if duration_1rev > 0 and duration_1rev < MAX_1REV_US:
            valid_duration = duration_1rev
            new_data_flag = True
            
    t_idx = (t_idx + 1) % 3

sensor.irq(trigger=machine.Pin.IRQ_FALLING, handler=calculate_speed)

# --- メインループ（重い処理と描画） ---
while True:
    # 1. 新しいパルスデータが来た時だけ計算する
    if new_data_flag:
        duration = valid_duration
        new_data_flag = False
        
        if start_measuring_time == 0:
            start_measuring_time = time.ticks_ms()
            
        rps = 1000000.0 / duration
        raw_speed = (CIRCUMFERENCE * rps) * 3.6
        
        # 配列の操作（メモリ消費）はここで行うので安全
        history.append(raw_speed)
        if len(history) > 5:
            history.pop(0)
        
        sorted_hist = sorted(history)
        display_speed = sorted_hist[len(sorted_hist)//2]

        if display_speed <= STOP_SPEED_THRESHOLD and stop_time_ms == 0:
            stop_time_ms = time.ticks_ms()
        elif display_speed > STOP_SPEED_THRESHOLD:
            stop_time_ms = 0

        if stop_time_ms != 0 and not reset_max_on_next_speed and time.ticks_diff(time.ticks_ms(), stop_time_ms) >= MAX_RESET_AFTER_STOP_MS:
            reset_max_on_next_speed = True

        if reset_max_on_next_speed and display_speed >= DISPLAY_SPEED_RESET_THRESHOLD:
            max_speed = 0.0
            reset_max_on_next_speed = False

        # MAX_UPDATE_DELAY_MS経過後のMAX値更新
        if start_measuring_time > 0 and time.ticks_diff(time.ticks_ms(), start_measuring_time) > MAX_UPDATE_DELAY_MS:
            if display_speed > max_speed:
                max_speed = display_speed

    # 2. 停止判定（最後のパルスから一定時間経過）
    if time.ticks_diff(time.ticks_us(), last_pulse_time) > STOP_TIMEOUT_US:
        display_speed = 0.0
        start_measuring_time = 0
        history = []
        if stop_time_ms == 0:
            stop_time_ms = time.ticks_ms()
        elif time.ticks_diff(time.ticks_ms(), stop_time_ms) >= MAX_RESET_AFTER_STOP_MS:
            reset_max_on_next_speed = True
    elif display_speed > STOP_SPEED_THRESHOLD:
        stop_time_ms = 0
        
    # 3. 画面の描画
    oled.fill(0)
    
    speed_str = "{:>4.1f}".format(display_speed)
    big_text(speed_str, 0, 16)
    oled.text("km/h", 70, 20)
    
    if max_speed == 0.0:
        max_display = "MAX: ---"
    else:
        max_str = "{:.1f}".format(max_speed)
        max_display = f"MAX: {max_str}"
    max_x = 128 - len(max_display) * 8
    oled.text(max_display, max_x, 0)

    if stop_time_ms != 0:
        remaining_ms = MAX_RESET_AFTER_STOP_MS - time.ticks_diff(time.ticks_ms(), stop_time_ms)
        if remaining_ms < 0:
            remaining_ms = 0
        remaining_sec = remaining_ms // 1000
        countdown_display = "{}s".format(remaining_sec)
        countdown_x = 128 - len(countdown_display) * 8
        oled.text(countdown_display, countdown_x, 24)

    oled.show()
    time.sleep(0.1) # 更新頻度を0.1秒(10FPS)にしてレスポンス向上
