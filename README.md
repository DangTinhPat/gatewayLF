# gatewayLF — tầng dưới Hybrid Control Runtime

Firmware cho ESP32-S3 N16R8: đọc MPU6500, nhận HCR1 từ Luckfox qua UART0 và
điều khiển sáu BL4012 qua hai bus UART. Đây là firmware MCU hybrid độc lập;
logic đứng dậy và model chạy trên LF.

- LF UART2 RX/TX GPIO1_B3/B2 nối lần lượt ESP GPIO43/TX0 và GPIO44/RX0.
- UART0: 921600 baud, 8N1, HCR1 nhị phân.
- Bus động cơ trái: UART2/GPIO15; phải: UART1/GPIO17.
- Phải làm MOTOR_HOME sau mỗi lần khởi động trước khi cho phép ARM.

Bus motor dùng module `joint_bus.c` từ bản BL4012e/3in1 đã được người dùng
kiểm trên motor; bản ghép HCR hiện tại vẫn cần kiểm lại trên phần cứng.
Xem [kiểm chứng bản kế thừa](firmware/esp32/BL4012E_REUSE_VALIDATION.md).

## Build

Cài PlatformIO Core, sau đó:

```sh
pio run -e esp32-s3-hybrid-mcu
```

Chọn đường nạp rõ ràng theo đúng board trước khi flash. Bản build mặc định
không chứa thông tin Wi-Fi hoặc mật khẩu OTA. Muốn bật Wi-Fi, sao chép
`HcrWifiSecrets.example.hpp` thành `HcrWifiSecrets.hpp` và điền mạng 2.4 GHz.
Muốn bật OTA, tạo `OtaSecrets.hpp` với MD5 của mật khẩu OTA; giữ cả file này
và mật khẩu ngoài Git. Không commit credential.

Xem [mô tả MCU](firmware/esp32/HYBRID_MCU.md),
[giao thức HCR1](firmware/esp32/HCR1_PROTOCOL.md),
[ghép driver](firmware/esp32/MOTOR_INTEGRATION.md), và
[OTA](firmware/esp32/OTA.md). Tài liệu phân biệt kiểm tra build/mock với
phép đo thật trên robot.
