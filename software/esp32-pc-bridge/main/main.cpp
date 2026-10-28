#include "bridge_core.hpp"
#include "sdkconfig.h"
#include "wire_v2.hpp"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>

#include "esp_event.h"
#include "esp_idf_version.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "mbedtls/md.h"

#if !CONFIG_IDF_TARGET_ESP32C3
#error "This firmware targets ESP32-C3 only"
#endif

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 5, 0)
#error "This project requires ESP-IDF 5.5 or newer"
#endif

#if CONFIG_BRIDGE_ROLE_MOCK_TX
#include "mock_foc.hpp"
#elif CONFIG_BRIDGE_OUTPUT_USB
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#else
#include "driver/uart.h"
#endif

extern "C" {
volatile esp_err_t bridge_init_status = ESP_OK;
}

namespace {

esp_err_t start_radio() {
  esp_err_t error = esp_netif_init();
  if (error != ESP_OK)
    return error;
  error = esp_event_loop_create_default();
  if (error != ESP_OK)
    return error;
  wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
  config.nvs_enable = 0;
  error = esp_wifi_init(&config);
  if (error != ESP_OK)
    return error;
  error = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  if (error != ESP_OK)
    return error;
  error = esp_wifi_set_mode(WIFI_MODE_STA);
  if (error != ESP_OK)
    return error;
#if CONFIG_BRIDGE_ROLE_MOCK_TX
  bridge::Mac source{};
  if (!bridge::parse_mac(CONFIG_BRIDGE_SOURCE_MAC, source) ||
      !bridge::unicast_mac(source)) {
    return ESP_ERR_INVALID_ARG;
  }
  error = esp_wifi_set_mac(WIFI_IF_STA, source.data());
  if (error != ESP_OK)
    return error;
#endif
  error = esp_wifi_start();
  if (error != ESP_OK)
    return error;
  error = esp_wifi_set_ps(WIFI_PS_NONE);
  if (error != ESP_OK)
    return error;
  error = esp_wifi_set_channel(CONFIG_BRIDGE_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (error != ESP_OK)
    return error;
  return esp_now_init();
}

#if CONFIG_BRIDGE_ROLE_MOCK_TX

esp_err_t run_mock() {
  bridge::Mac destination{};
  if (!bridge::parse_mac(CONFIG_BRIDGE_DESTINATION_MAC, destination)) {
    return ESP_ERR_INVALID_ARG;
  }
  const bridge::Mac broadcast{0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  if (destination != broadcast && !bridge::unicast_mac(destination)) {
    return ESP_ERR_INVALID_ARG;
  }
  esp_now_peer_info_t peer{};
  std::copy(destination.begin(), destination.end(), peer.peer_addr);
  peer.channel = CONFIG_BRIDGE_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const auto error = esp_now_add_peer(&peer);
  if (error != ESP_OK)
    return error;
  const auto session = esp_random();
  TickType_t wake = xTaskGetTickCount();
  for (std::uint32_t step = 0;; ++step) {
    const auto packet = bridge::mock::scenario(session, step);
    esp_now_send(destination.data(), packet.bytes.data(), packet.size);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(bridge::mock::kStepMs));
  }
}

#elif !CONFIG_BRIDGE_ROLE_PC && !CONFIG_BRIDGE_ROLE_MOTOR

struct Datagram {
  bridge::Mac source{};
  std::array<std::uint8_t, bridge::kMaxPacket> bytes{};
  std::uint16_t size = 0;
};

StaticQueue_t radio_queue_control;
alignas(Datagram) std::uint8_t
    radio_queue_storage[bridge::kQueueDepth * sizeof(Datagram)];
QueueHandle_t radio_queue = nullptr;
std::uint32_t radio_queue_dropped = 0;

void on_receive(const esp_now_recv_info_t *info, const std::uint8_t *bytes,
                int size) {
  if (info == nullptr || info->src_addr == nullptr || bytes == nullptr ||
      size < static_cast<int>(bridge::kHeaderSize + 3) ||
      size > static_cast<int>(bridge::kMaxPacket)) {
    return;
  }
  Datagram datagram;
  std::copy_n(info->src_addr, datagram.source.size(), datagram.source.begin());
  datagram.size = static_cast<std::uint16_t>(size);
  std::memcpy(datagram.bytes.data(), bytes, datagram.size);
  if (xQueueSend(radio_queue, &datagram, 0) != pdTRUE) {
    ++radio_queue_dropped;
  }
}

class Output {
public:
  esp_err_t init() {
#if CONFIG_BRIDGE_OUTPUT_USB
    usb_serial_jtag_driver_config_t config{};
    config.tx_buffer_size = 2048;
    config.rx_buffer_size = 256;
    return usb_serial_jtag_driver_install(&config);
#else
    uart_config_t config{};
    config.baud_rate = 115200;
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    config.source_clk = UART_SCLK_DEFAULT;
    auto error = uart_param_config(UART_NUM_0, &config);
    if (error != ESP_OK)
      return error;
    error = uart_set_pin(UART_NUM_0, CONFIG_BRIDGE_UART_TX_GPIO,
                         CONFIG_BRIDGE_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                         UART_PIN_NO_CHANGE);
    if (error != ESP_OK)
      return error;
    return uart_driver_install(UART_NUM_0, 256, 0, 0, nullptr, 0);
#endif
  }

  std::size_t write(const std::uint8_t *bytes, std::size_t size) {
#if CONFIG_BRIDGE_OUTPUT_USB
    const int written = usb_serial_jtag_write_bytes(bytes, size, 0);
#else
    const int written =
        uart_tx_chars(UART_NUM_0, reinterpret_cast<const char *>(bytes),
                      static_cast<std::uint32_t>(size));
#endif
    return written > 0 ? static_cast<std::size_t>(written) : 0;
  }
};

esp_err_t run_receiver() {
  bridge::Mac source{};
  if (!bridge::parse_mac(CONFIG_BRIDGE_SOURCE_MAC, source) ||
      !bridge::unicast_mac(source)) {
    return ESP_ERR_INVALID_ARG;
  }
  Output output;
  auto error = output.init();
  if (error != ESP_OK)
    return error;
  radio_queue = xQueueCreateStatic(bridge::kQueueDepth, sizeof(Datagram),
                                   radio_queue_storage, &radio_queue_control);
  if (radio_queue == nullptr)
    return ESP_ERR_NO_MEM;
  error = esp_now_register_recv_cb(on_receive);
  if (error != ESP_OK)
    return error;
  static bridge::Receiver receiver(source);
  for (;;) {
    Datagram datagram;
    for (std::size_t i = 0; i < bridge::kQueueDepth; ++i) {
      if (xQueueReceive(radio_queue, &datagram, 0) != pdTRUE)
        break;
      receiver.receive(datagram.source, datagram.bytes.data(), datagram.size,
                       static_cast<std::uint64_t>(esp_timer_get_time() / 1000));
    }
    receiver.pump(output);
    vTaskDelay(1);
  }
}

#endif

#if CONFIG_BRIDGE_ROLE_PC || CONFIG_BRIDGE_ROLE_MOTOR
constexpr std::size_t kRadioTagSize = 16;
constexpr std::size_t kMaxRadioPacket = bridge::v2::kMaxPacket + kRadioTagSize;
struct V2Datagram {
  bridge::Mac source{};
  std::array<std::uint8_t, kMaxRadioPacket> data{};
  std::uint16_t size = 0;
};
StaticQueue_t v2q_control;
alignas(V2Datagram) std::uint8_t v2q_storage[8 * sizeof(V2Datagram)];
QueueHandle_t v2q = nullptr;
void v2_receive(const esp_now_recv_info_t *info, const std::uint8_t *data,
                int len) {
  if (!info || !info->src_addr || !data ||
      len < static_cast<int>(bridge::v2::kHeaderSize + 2 + kRadioTagSize) ||
      len > static_cast<int>(kMaxRadioPacket))
    return;
  V2Datagram d;
  std::copy_n(info->src_addr, 6, d.source.begin());
  d.size = static_cast<std::uint16_t>(len);
  std::memcpy(d.data.data(), data, d.size);
  (void)xQueueSend(v2q, &d, 0);
}
class V2Port {
public:
  esp_err_t init() {
#if CONFIG_BRIDGE_ROLE_PC && CONFIG_BRIDGE_OUTPUT_USB
    usb_serial_jtag_driver_config_t c{};
    c.tx_buffer_size = 2048;
    c.rx_buffer_size = 512;
    return usb_serial_jtag_driver_install(&c);
#elif CONFIG_BRIDGE_ROLE_PC
    const auto port = UART_NUM_0;
#else
    const auto port = UART_NUM_1;
#endif
#if !(CONFIG_BRIDGE_ROLE_PC && CONFIG_BRIDGE_OUTPUT_USB)
    uart_config_t c{};
    c.baud_rate = 115200;
    c.data_bits = UART_DATA_8_BITS;
    c.parity = UART_PARITY_DISABLE;
    c.stop_bits = UART_STOP_BITS_1;
    c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    c.source_clk = UART_SCLK_DEFAULT;
    esp_err_t e = uart_param_config(port, &c);
    if (e != ESP_OK)
      return e;
#if CONFIG_BRIDGE_ROLE_PC
    e = uart_set_pin(port, CONFIG_BRIDGE_UART_TX_GPIO,
                     CONFIG_BRIDGE_UART_RX_GPIO, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
#else
    e = uart_set_pin(port, CONFIG_BRIDGE_MOTOR_TX_GPIO,
                     CONFIG_BRIDGE_MOTOR_RX_GPIO, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
#endif
    if (e != ESP_OK)
      return e;
    return uart_driver_install(port, 1024, 1024, 0, nullptr, 0);
#endif
  }
  int read(std::uint8_t *b, std::size_t n) {
#if CONFIG_BRIDGE_ROLE_PC && CONFIG_BRIDGE_OUTPUT_USB
    return usb_serial_jtag_read_bytes(b, n, 0);
#elif CONFIG_BRIDGE_ROLE_PC
    return uart_read_bytes(UART_NUM_0, b, n, 0);
#else
    return uart_read_bytes(UART_NUM_1, b, n, 0);
#endif
  }
  int write(const std::uint8_t *b, std::size_t n) {
#if CONFIG_BRIDGE_ROLE_PC && CONFIG_BRIDGE_OUTPUT_USB
    return usb_serial_jtag_write_bytes(b, n, 0);
#elif CONFIG_BRIDGE_ROLE_PC
    return uart_write_bytes(UART_NUM_0, reinterpret_cast<const char *>(b), n);
#else
    return uart_write_bytes(UART_NUM_1, reinterpret_cast<const char *>(b), n);
#endif
  }
};
bool write_packet(V2Port &port, const std::uint8_t *bytes, std::size_t size) {
  const auto end = esp_timer_get_time() + 100000;
  std::size_t off = 0;
  while (off < size && esp_timer_get_time() < end) {
    int n = port.write(bytes + off, size - off);
    if (n > 0)
      off += static_cast<std::size_t>(n);
    else
      vTaskDelay(1);
  }
  return off == size;
}
struct OutPacket {
  bridge::v2::Packet p{};
  std::uint64_t deadline = 0;
};
class OutQueue {
public:
  bool push(const bridge::v2::Packet &p, bool priority, std::uint64_t now) {
    auto &q = priority ? control_ : telemetry_;
    auto &count = priority ? cn_ : tn_;
    auto &tail = priority ? ct_ : tt_;
    if (count == 8)
      return false;
    q[tail] = OutPacket{p, now + 100};
    tail = (tail + 1) % 8;
    ++count;
    return true;
  }
  bool pop(std::uint64_t now, OutPacket &o) {
    while (cn_ && control_[ch_].deadline <= now) {
      ch_ = (ch_ + 1) % 8;
      --cn_;
    }
    while (tn_ && telemetry_[th_].deadline <= now) {
      th_ = (th_ + 1) % 8;
      --tn_;
    }
    auto &q = cn_ ? control_ : telemetry_;
    auto &h = cn_ ? ch_ : th_;
    auto &c = cn_ ? cn_ : tn_;
    if (!c)
      return false;
    o = q[h];
    h = (h + 1) % 8;
    --c;
    return true;
  }

private:
  std::array<OutPacket, 8> control_{}, telemetry_{};
  std::size_t ch_ = 0, ct_ = 0, th_ = 0, tt_ = 0, cn_ = 0, tn_ = 0;
};
bool constant_time_equal(const std::uint8_t *a, const std::uint8_t *b,
                         std::size_t size) {
  std::uint8_t diff = 0;
  for (std::size_t i = 0; i < size; ++i)
    diff |= static_cast<std::uint8_t>(a[i] ^ b[i]);
  return diff == 0;
}
bool hmac_sha256(const std::uint8_t *key, std::size_t key_size,
                 const std::uint8_t *data, std::size_t data_size,
                 std::array<std::uint8_t, 32> &out) {
  const auto *info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  return info &&
         mbedtls_md_hmac(info, key, key_size, data, data_size, out.data()) == 0;
}
bool hmac_self_test() {
  static constexpr std::uint8_t key[] = {'k', 'e', 'y'};
  static constexpr char message[] =
      "The quick brown fox jumps over the lazy dog";
  static constexpr std::array<std::uint8_t, 32> expected{
      0xf7, 0xbc, 0x83, 0xf4, 0x30, 0x53, 0x84, 0x24, 0xb1, 0x32, 0x98,
      0xe6, 0xaa, 0x6f, 0xb1, 0x43, 0xef, 0x4d, 0x59, 0xa1, 0x49, 0x46,
      0x17, 0x59, 0x97, 0x47, 0x9d, 0xbc, 0x2d, 0x1a, 0x3c, 0xd8};
  std::array<std::uint8_t, 32> actual{};
  return hmac_sha256(key, sizeof(key),
                     reinterpret_cast<const std::uint8_t *>(message),
                     sizeof(message) - 1, actual) &&
         constant_time_equal(actual.data(), expected.data(), expected.size());
}
bool authenticate_radio(const std::uint8_t *data, std::size_t size,
                        std::size_t &frame_size) {
  frame_size = 0;
  if (!data || size < bridge::v2::kHeaderSize + 2 + kRadioTagSize ||
      size > kMaxRadioPacket)
    return false;
  const std::size_t raw_size = size - kRadioTagSize;
  std::array<std::uint8_t, 32> mac{};
  if (!hmac_sha256(
          reinterpret_cast<const std::uint8_t *>(CONFIG_BRIDGE_WIFI_LMK), 16,
          data, raw_size, mac))
    return false;
  if (!constant_time_equal(mac.data(), data + raw_size, kRadioTagSize))
    return false;
  frame_size = raw_size;
  return true;
}
int send_radio(const bridge::Mac &peer, const bridge::v2::Packet &packet) {
  if (packet.size > bridge::v2::kMaxPacket)
    return ESP_ERR_INVALID_SIZE;
  std::array<std::uint8_t, kMaxRadioPacket> wire{};
  std::memcpy(wire.data(), packet.bytes.data(), packet.size);
  std::array<std::uint8_t, 32> mac{};
  if (!hmac_sha256(
          reinterpret_cast<const std::uint8_t *>(CONFIG_BRIDGE_WIFI_LMK), 16,
          wire.data(), packet.size, mac))
    return ESP_FAIL;
  std::memcpy(wire.data() + packet.size, mac.data(), kRadioTagSize);
  return esp_now_send(peer.data(), wire.data(), packet.size + kRadioTagSize);
}
bool v2_setup(bridge::Mac &peer) {
  if (!hmac_self_test())
    return false;
#if CONFIG_BRIDGE_ROLE_MOTOR
  const auto allowed = [](int pin) {
    return pin >= 0 && pin <= 21 && pin != 2 && pin != 8 && pin != 9 &&
           !(pin >= 12 && pin <= 19);
  };
  if (CONFIG_BRIDGE_MOTOR_TX_GPIO == CONFIG_BRIDGE_MOTOR_RX_GPIO ||
      !allowed(CONFIG_BRIDGE_MOTOR_TX_GPIO) ||
      !allowed(CONFIG_BRIDGE_MOTOR_RX_GPIO))
    return false;
#endif
  if (std::strlen(CONFIG_BRIDGE_WIFI_PMK) != 16 ||
      std::strlen(CONFIG_BRIDGE_WIFI_LMK) != 16 ||
      !bridge::parse_mac(CONFIG_BRIDGE_PEER_MAC, peer) ||
      !bridge::unicast_mac(peer))
    return false;
  return true;
}
esp_err_t start_secure_radio(const bridge::Mac &peer) {
  auto e = start_radio();
  if (e != ESP_OK)
    return e;
  e = esp_now_set_pmk(
      reinterpret_cast<const std::uint8_t *>(CONFIG_BRIDGE_WIFI_PMK));
  if (e != ESP_OK)
    return e;
  esp_now_peer_info_t p{};
  std::copy(peer.begin(), peer.end(), p.peer_addr);
  p.channel = CONFIG_BRIDGE_CHANNEL;
  p.ifidx = WIFI_IF_STA;
  p.encrypt = true;
  const char *lmk = CONFIG_BRIDGE_WIFI_LMK;
  const std::size_t lmk_len = std::strlen(lmk);
  for (std::size_t i = 0; i < sizeof(p.lmk); ++i)
    p.lmk[i] = i < lmk_len ? static_cast<std::uint8_t>(lmk[i]) : 0;
  e = esp_now_add_peer(&p);
  if (e != ESP_OK)
    return e;
  v2q = xQueueCreateStatic(8, sizeof(V2Datagram), v2q_storage, &v2q_control);
  if (!v2q)
    return ESP_ERR_NO_MEM;
  return esp_now_register_recv_cb(v2_receive);
}
bool encode_v2(const bridge::v2::Frame &f, bridge::v2::Packet &p) {
  return bridge::v2::encode(f, p);
}
bool kind_pc_to_motor(std::uint8_t k) {
  return k == bridge::v2::hello || k == bridge::v2::request;
}
bool kind_motor_to_pc(std::uint8_t k) {
  return k == bridge::v2::identity || k == bridge::v2::response ||
         k == bridge::v2::telemetry;
}
#if CONFIG_BRIDGE_ROLE_PC
esp_err_t run_pc() {
  bridge::Mac peer{};
  if (!v2_setup(peer))
    return ESP_ERR_INVALID_STATE;
  auto e = start_secure_radio(peer);
  if (e != ESP_OK)
    return e;
  V2Port port;
  e = port.init();
  if (e != ESP_OK)
    return e;
  bridge::v2::Decoder decoder;
  static OutQueue outgoing, to_host;
  std::array<std::uint8_t, 128> input{};
  V2Datagram d;
  std::uint64_t last_input = esp_timer_get_time() / 1000;
  for (;;) {
    auto now = static_cast<std::uint64_t>(esp_timer_get_time() / 1000);
    int n = port.read(input.data(), input.size());
    if (n > 0)
      last_input = now;
    else if (now - last_input >= 100) {
      decoder.reset();
      last_input = now;
    }
    for (int i = 0; i < n; i++) {
      bridge::v2::Frame f;
      if (!decoder.push(input[i], f))
        continue;
      if (f.kind == bridge::v2::bridge_hello) {
        char msg[80];
        int z = std::snprintf(
            msg, sizeof(msg),
            "role=pc protocol=2 secure=1 peer=%02x:%02x:%02x:%02x:%02x:%02x",
            peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);
        bridge::v2::Frame r{bridge::v2::bridge_info,
                            f.session,
                            f.boot,
                            f.request,
                            f.sequence,
                            f.permit,
                            reinterpret_cast<const std::uint8_t *>(msg),
                            static_cast<std::size_t>(z)};
        bridge::v2::Packet p;
        if (encode_v2(r, p) && !write_packet(port, p.bytes.data(), p.size))
          return ESP_FAIL;
      } else if (kind_pc_to_motor(f.kind)) {
        bridge::v2::Packet p;
        if (encode_v2(f, p))
          outgoing.push(p, true, now);
      }
    }
    for (std::size_t received = 0;
         received < 8 && xQueueReceive(v2q, &d, 0) == pdTRUE; ++received) {
      if (d.source != peer)
        continue;
      std::size_t frame_size = 0;
      if (!authenticate_radio(d.data.data(), d.size, frame_size))
        continue;
      bridge::v2::Frame f;
      if (!bridge::v2::decode(d.data.data(), frame_size, f) ||
          !kind_motor_to_pc(f.kind))
        continue;
      bridge::v2::Packet p;
      if (encode_v2(f, p))
        to_host.push(p, f.kind != bridge::v2::telemetry, now);
    }
    OutPacket host;
    if (to_host.pop(now, host) &&
        !write_packet(port, host.p.bytes.data(), host.p.size))
      return ESP_FAIL;
    OutPacket p;
    if (outgoing.pop(now, p))
      send_radio(peer, p.p);
    vTaskDelay(1);
  }
}
#else
esp_err_t run_motor() {
  bridge::Mac peer{};
  if (!v2_setup(peer))
    return ESP_ERR_INVALID_STATE;
  auto e = start_secure_radio(peer);
  if (e != ESP_OK)
    return e;
  V2Port port;
  e = port.init();
  if (e != ESP_OK)
    return e;
  bridge::v2::Decoder decoder;
  static OutQueue outgoing;
  std::array<std::uint8_t, 128> input{};
  V2Datagram d;
  std::uint64_t last_input = esp_timer_get_time() / 1000;
  for (;;) {
    auto now = static_cast<std::uint64_t>(esp_timer_get_time() / 1000);
    int n = port.read(input.data(), input.size());
    if (n > 0)
      last_input = now;
    else if (now - last_input >= 100) {
      decoder.reset();
      last_input = now;
    }
    for (int i = 0; i < n; i++) {
      bridge::v2::Frame f;
      if (!decoder.push(input[i], f))
        continue;
      if (kind_motor_to_pc(f.kind)) {
        bridge::v2::Packet p;
        if (encode_v2(f, p))
          outgoing.push(p, f.kind != bridge::v2::telemetry, now);
      }
    }
    for (std::size_t received = 0;
         received < 8 && xQueueReceive(v2q, &d, 0) == pdTRUE; ++received) {
      if (d.source != peer)
        continue;
      std::size_t frame_size = 0;
      if (!authenticate_radio(d.data.data(), d.size, frame_size))
        continue;
      bridge::v2::Frame f;
      if (!bridge::v2::decode(d.data.data(), frame_size, f) ||
          !kind_pc_to_motor(f.kind))
        continue;
      bridge::v2::Packet p;
      if (!encode_v2(f, p))
        continue;
      if (!write_packet(port, p.bytes.data(), p.size))
        return ESP_FAIL;
    }
    OutPacket p;
    if (outgoing.pop(now, p))
      send_radio(peer, p.p);
    vTaskDelay(1);
  }
}
#endif
#endif

} // namespace

extern "C" void app_main() {
#if CONFIG_BRIDGE_ROLE_PC
  bridge_init_status = run_pc();
#elif CONFIG_BRIDGE_ROLE_MOTOR
  bridge_init_status = run_motor();
#elif CONFIG_BRIDGE_ROLE_MOCK_TX
  bridge_init_status = start_radio();
  if (bridge_init_status == ESP_OK)
    bridge_init_status = run_mock();
#else
  bridge_init_status = start_radio();
  if (bridge_init_status == ESP_OK)
    bridge_init_status = run_receiver();
#endif
}
