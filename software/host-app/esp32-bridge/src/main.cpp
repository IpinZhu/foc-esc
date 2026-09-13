// ESP32-C3 有线桥 v0 —— 方案验证（需求文档 TR-B5/TR-B7，里程碑 M5a）
//
// 拓扑:  PC --USB-CDC--> ESP32-C3 --UART1--> FOC 板(USART3)
//
// 行为: 双向纯透传，对协议字节零修改、零注入。
// 未来无线化（M5b）时，成帧/ACK 只出现在两片 ESP32 的 2.4G 链路上，
// 本固件的 CDC 与 UART 段保持透传不变。
//
// 编译变体 BRIDGE_MODE_RESPONDER=1（env:esp32c3_responder）：不做透传，
// 改为在 CDC 口上模拟 FOC 的 ASCII 协议应答（status/param-status/ok），
// 用于 FOC 板卡缺位时的 PC<->C3 链路验证。
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

#if BRIDGE_MODE_RESPONDER

// ---- 应答器模式: 模拟 FOC 的最小 ASCII 协议 ----
// status        -> 一行遥测（seq 递增，24V/Idle/零电流）
// param status  -> 参数存储状态行
// 其它非空行    -> ok
static uint32_t responderSeq = 100;
static String pendingLine;

static void handleLine(String& line) {
  line.trim();
  if (line == "status") {
    responderSeq++;
    pcSerial.print("seq=");
    pcSerial.print(static_cast<unsigned>(responderSeq));
    pcSerial.print(
        " state=1 faults=00000000 ia=0.10 ib=-0.05 ic=-0.05 "
        "id=0.00 iq=0.00 vbus=24.00 vel=0.00 temp=27.0\r\n");
  } else if (line == "param status") {
    pcSerial.print(
        "param-status valid=0 source=defaults generation=0 dirty=0 "
        "restart-required=0\r\n");
  } else if (line.length() > 0) {
    pcSerial.print("ok\r\n");
  }
}

static void responderPump() {
  while (pcSerial.available() > 0) {
    char ch = static_cast<char>(pcSerial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      handleLine(pendingLine);
      pendingLine = "";
    } else if (pendingLine.length() < 96) {
      pendingLine += ch;
    }
  }
}

void loop() {
  responderPump();
#ifdef LED_BUILTIN
  digitalWrite(LED_BUILTIN, (millis() / 500) & 1 ? HIGH : LOW);
#endif
  delayMicroseconds(200);
}

#else

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

#endif  // BRIDGE_MODE_RESPONDER
