// ESP32-C3 有线桥 v0 —— 方案验证（需求文档 TR-B5/TR-B7，里程碑 M5a）
//
// 拓扑:  PC --USB-CDC--> ESP32-C3 --UART1--> FOC 板(USART3)
//
// 行为: 双向纯透传，对协议字节零修改、零注入。
// 未来无线化（M5b）时，成帧/ACK 只出现在两片 ESP32 的 2.4G 链路上，
// 本固件的 CDC 与 UART 段保持透传不变。
//
// 接线（共地！）:
//   C3 GPIO4 (UART1 TX) -> FOC PB11 (USART3 RX)
//   C3 GPIO5 (UART1 RX) <- FOC PB10 (USART3 TX)
//   C3 GND              <-> FOC GND
//
// 注意: C3 复位时 USB 会重新枚举（USB-Serial-JTAG 特性），上位机需容忍。

#include <Arduino.h>

static constexpr uint32_t LINK_BAUD = 115200;
static constexpr int LINK_TX_PIN = 4;
static constexpr int LINK_RX_PIN = 5;
static constexpr size_t CHUNK_SIZE = 256;

// PC 侧: JTAG 型为 HWCDC、UART0 型为 HardwareSerial，共同基类是 Stream
static Stream& pcSerial = Serial;
// FOC 侧: UART1
static HardwareSerial linkSerial(1);

static uint8_t transferChunk[CHUNK_SIZE];

// 把 from 里当前可读的字节搬到 to，单次至多 CHUNK_SIZE。
// 使用逐字节读打包成块写，兼容 HWCDC / HardwareSerial 两种 Stream 实现。
static void pumpStream(Stream& from, Stream& to) {
  int available = from.available();
  if (available <= 0) {
    return;
  }
  if (available > static_cast<int>(CHUNK_SIZE)) {
    available = static_cast<int>(CHUNK_SIZE);
  }
  int count = 0;
  while (count < available) {
    int byte = from.read();
    if (byte < 0) {
      break;
    }
    transferChunk[count++] = static_cast<uint8_t>(byte);
  }
  if (count > 0) {
    to.write(transferChunk, static_cast<size_t>(count));
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  linkSerial.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  // PC 侧不等待主机连接：桥必须无头可用。
  // Serial 在两种编译变体下自动映射：JTAG 型 -> HWCDC；UART0 型 -> HardwareSerial(0)。
  Serial.begin(LINK_BAUD);
}

void loop() {
  pumpStream(pcSerial, linkSerial);
  pumpStream(linkSerial, pcSerial);

  // 心跳灯: 2 Hz 常亮闪烁表示固件在跑; 有数据活动时常亮。
  static uint32_t lastActivity = 0;
  uint32_t now = millis();
  bool active = pcSerial.available() > 0 || linkSerial.available() > 0;
  if (active) {
    lastActivity = now;
  }
#ifdef LED_BUILTIN
  if (now - lastActivity < 150) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, (now / 500) & 1 ? HIGH : LOW);
  }
#endif

  // 无数据时稍作让步，降低空转功耗与占用
  if (!active) {
    delayMicroseconds(200);
  }
}
