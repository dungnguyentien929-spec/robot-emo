# Robot Emo

Robot để bàn biết nói chuyện bằng tiếng Việt, chạy trên ESP32-S3. Emo nghe bạn nói, trả lời bằng giọng nói,
chạy tới lui bằng bánh xe, đo nhịp tim, xem thời tiết và thể hiện cảm xúc bằng đôi mắt trên màn hình OLED.

Đây là bản fork của [78/xiaozhi-esp32](https://github.com/78/xiaozhi-esp32) (XiaoZhi AI, phiên bản 2.6.0).
Phần riêng của Robot Emo nằm trong [main/boards/robot-emo/](main/boards/robot-emo/); phần còn lại là code gốc
của XiaoZhi.

## Emo làm được gì

- **Trò chuyện tiếng Việt** qua máy chủ chính thức [xiaozhi.me](https://xiaozhi.me): nhận giọng nói trên đám mây,
  mô hình ngôn ngữ lớn trả lời, đọc lại bằng giọng nói. Tính cách của Emo được cấu hình trên xiaozhi.me.
- **Từ đánh thức** "Hi ESP" (WakeNet chạy ngay trên chip, không cần mạng).
- **Di chuyển** bằng hai động cơ qua L298N: tiến, lùi, quay trái, quay phải, dừng. Có nhiều lớp an toàn: chân
  động cơ ở mức LOW khi khởi động, tự dừng sau tối đa 5 giây, dừng khi mất Wi-Fi.
- **Đo nhịp tim** bằng cảm biến MAX30102 (chỉ để tham khảo, không dùng cho y tế).
- **Thời tiết** hiện tại, hôm nay và ngày mai cho Thủ Đức, lấy từ [Open-Meteo](https://open-meteo.com) (miễn phí,
  không cần API key), lưu bộ nhớ đệm và làm mới mỗi 15 phút.
- **Đôi mắt hoạt hình** trên OLED 128x64: chớp mắt, liếc nhìn, đổi biểu cảm theo cảm xúc của câu trả lời.

Các chức năng của robot được đưa cho mô hình ngôn ngữ dưới dạng công cụ MCP, nên bạn chỉ cần nói tự nhiên,
ví dụ "đi tới 3 giây", "mai có mưa không", "đo nhịp tim cho tôi".

| Công cụ MCP | Chức năng |
| --- | --- |
| `self.robot.move` | Di chuyển theo hướng `forward`, `backward`, `left`, `right` trong 0,5 đến 5 giây |
| `self.robot.stop` | Dừng ngay |
| `self.sensor.get_heart_rate` | Đọc nhịp tim (BPM) |
| `self.weather.get` | Thời tiết Thủ Đức hiện tại, hôm nay và ngày mai |

## Phần cứng

| Linh kiện | Kết nối (GPIO) |
| --- | --- |
| ESP32-S3 N16R8 (16 MB flash, 8 MB PSRAM octal) | |
| Micro INMP441 | WS 4, SCK 5, DIN 6 |
| Ampli loa MAX98357A + loa | DIN 7, BCLK 15, LRC 16 |
| OLED SSD1306 128x64 (I2C, địa chỉ 0x3C) | SDA 41, SCL 42 |
| Driver động cơ L298N (không dùng chân ENA/ENB) | IN1 11, IN2 12, IN3 13, IN4 14 |
| Cảm biến nhịp tim MAX30102 (I2C, địa chỉ 0x57) | SDA 8, SCL 9 |
| Nút BOOT | 0 |

GPIO 35 đến 37 thuộc về PSRAM octal, không dùng được. Chiều quay của từng động cơ chỉnh được trong code
(`kInvertMotorA`, `kInvertMotorB`) mà không cần đổi dây.

### Lưu ý về nguồn điện

Động cơ và loa kéo dòng đỉnh lớn. Nếu ESP32 dùng chung nguồn yếu với chúng, điện áp sẽ sụt và chip tự khởi động
lại (trong log có dòng `Brownout detector was triggered`). Nên làm:

- Cấp cho ESP32 một nguồn 5 V ổn định riêng (mạch tăng áp hoặc hạ áp từ 2 A trở lên), không lấy từ chân "5V" của
  module động cơ.
- Không cấp nguồn cho MAX98357A từ chân 3V3 của board ESP32; lấy từ 5 V hoặc trực tiếp từ pin.
- Nối chung GND, thêm tụ 470 đến 1000 µF ở đầu vào driver động cơ và tụ 0,1 µF trên mỗi động cơ.

Firmware có sẵn vài biện pháp giảm sụt áp: động cơ chỉ chạy sau khi robot nói xong, nghỉ 100 ms trước khi đổi
chiều, và âm lượng loa tối đa là 50.

## Build và nạp firmware

Cần [ESP-IDF v6.0.3](https://docs.espressif.com/projects/esp-idf/). Sau khi kích hoạt môi trường ESP-IDF:

```powershell
python scripts\build.py robot-emo --name robot-emo --language vi-VN
idf.py -p COMx flash monitor
```

Thay `COMx` bằng cổng của board. Lần đầu chạy, robot phát Wi-Fi để bạn nhập mạng nhà, sau đó đọc mã kích hoạt
để bạn thêm thiết bị trên [xiaozhi.me](https://xiaozhi.me). Vị trí thời tiết sửa ở `kWeatherPlace` và
`kWeatherUrl` trong [robot_emo_board.cc](main/boards/robot-emo/robot_emo_board.cc).

Chi tiết kỹ thuật (thuật toán đo nhịp tim, các lỗi đặc thù của module MAX30102, thiết kế đôi mắt, bộ nhớ) nằm
trong [README của board](main/boards/robot-emo/README.md).

## Cấu trúc thư mục chính

```
main/boards/robot-emo/
  robot_emo_board.cc     board: động cơ, nhịp tim, thời tiết, công cụ MCP
  weather_report.*       đọc và định dạng dữ liệu Open-Meteo
  eye_renderer.*         vẽ đôi mắt (C++ thuần, test được trên máy tính)
  eyes_display.*         hiển thị đôi mắt trên OLED
  config.h, config.json  chân GPIO và cấu hình build
scripts/tests/           test trên máy tính cho phần thời tiết và đôi mắt
docs/superpowers/        bản thiết kế và kế hoạch của từng tính năng
```

## Tình trạng

Đã chạy trên phần cứng: trò chuyện tiếng Việt, từ đánh thức "Hi ESP", di chuyển, đo nhịp tim, thời tiết. Đôi mắt
OLED mới được thêm, đang được kiểm tra trên phần cứng.

## Ghi công và giấy phép

Dựa trên [XiaoZhi AI (78/xiaozhi-esp32)](https://github.com/78/xiaozhi-esp32), giấy phép MIT, bản quyền
Shenzhen Xinzhi Future Technology Co., Ltd. Phần sửa đổi của Robot Emo cũng phát hành theo giấy phép MIT; xem
[LICENSE](LICENSE). Hướng dẫn gốc của XiaoZhi: [tiếng Trung](README_zh.md), [tiếng Nhật](README_ja.md),
[bản tiếng Anh trên repo gốc](https://github.com/78/xiaozhi-esp32#readme).
