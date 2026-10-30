/*
 * ESP32-C3 ADC 线性度采集器
 *
 * 单一 ADC 通道（默认 GPIO3 / ADC1_CH3），12 dB 衰减、12 bit 分辨率。
 * 每 50 ms 做一次 N 点过采样，同时输出：
 *   - ADC 原始码（过采样均值，含两位小数）
 *   - 原始码的 min/max（观察噪声峰峰值）
 *   - 原始码标准差 σ（观察噪声有效值）
 *   - ESP-IDF 校准曲线（curve fitting）换算出的 mV
 *   - 用 min/max 原始码换算出的 mV（观察噪声折算到 mV）
 *
 * 串口协议（115200-8-N-1，一行一条，逗号分隔）：
 *   #FW,adc_linearity,2
 *   #CFG,pin=GPIO3,chan=3,atten=12,width=12,n=64,period_ms=50,cali=1
 *   #COL,t_ms,raw_x100,raw_min,raw_max,cal_mv,cal_min_mv,cal_max_mv,raw_std_x100
 *   D,<t_ms>,<raw_x100>,<min>,<max>,<mv>,<mv_min>,<mv_max>,<std_x100>
 *
 * 运行时命令（USB Serial/JTAG 输入，一行一条，见 "H"）：
 *   H            帮助
 *   S            打印当前 #CFG
 *   C <0-4>      切换 ADC1 通道 / GPIO
 *   A <0|2|6|12> 切换衰减（dB）
 *   N <1-256>    过采样点数
 *   P <10-5000>  采样周期（ms）
 *   B [n]        高速抓取 n 个原始码（默认 1024，最大 8192），用于噪声频谱分析：
 *                #BURST_BEGIN,<n>,<total_us>,<chan>,<atten>
 *                B,<i>,<raw>
 *                #BURST_END
 *
 * 主机端 GUI（host/gui.py）按 D 行解析，raw = raw_x100 / 100。
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

#define FW_VERSION       2
#define ADC_UNIT_ID      ADC_UNIT_1
#define ADC_CHAN_MIN     0
#define ADC_CHAN_MAX     4                 /* C3 ADC1: GPIO0~GPIO4 */
#define OVERSAMPLE_MIN   1
#define OVERSAMPLE_MAX   256
#define PERIOD_MS_MIN    10
#define PERIOD_MS_MAX    5000
#define BURST_DEFAULT    1024
#define BURST_MAX        8192

/* Wi-Fi 凭据放在 main/wifi_secrets.h（已 gitignore），仓库里只保留示例 */
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID        "your_ssid"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS        "your_password"
#endif
#define WIFI_UDP_PORT    5000
#define WIFI_BCAST_IP    "255.255.255.255"

typedef struct {
    int channel;                 /* ADC1 通道号，等于 GPIO 号（0~4） */
    adc_atten_t atten;
    int atten_db;
    int oversample;
    int period_ms;
} adc_cfg_t;

static const char *TAG = "adc_lin";

static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_cali = NULL;
static adc_cfg_t s_active = {
    .channel = 3, .atten = ADC_ATTEN_DB_12, .atten_db = 12,
    .oversample = 64, .period_ms = 50,
};

/* 命令任务只写 pending，主循环在两次采样之间应用，避免并发操作 ADC。 */
static volatile bool s_apply_req = false;
static volatile bool s_status_req = false;
static volatile bool s_burst_req = false;
static volatile int s_burst_n = BURST_DEFAULT;
static adc_cfg_t s_pending = {
    .channel = 3, .atten = ADC_ATTEN_DB_12, .atten_db = 12,
    .oversample = 64, .period_ms = 50,
};

/* ------------------------------------------------------------ Wi-Fi 负载 */
static volatile bool s_wifi_on = false;
static volatile bool s_wifi_inited = false;
static volatile uint32_t s_wifi_tx_pkts = 0;
static volatile uint32_t s_wifi_rx_pkts = 0;
static volatile uint32_t s_wifi_tx_bytes = 0;
static volatile uint32_t s_wifi_rx_bytes = 0;
static volatile uint32_t s_wifi_peer_ip = 0;
static volatile uint16_t s_wifi_peer_port = 0;
static volatile bool s_wifi_got_ip = false;
static volatile uint32_t s_wifi_gw_ip = 0;
static char s_wifi_ip[16] = "0.0.0.0";
static esp_netif_t *s_sta_netif = NULL;
static TaskHandle_t s_wifi_task = NULL;

/* ------------------------------------------------------------------ 校准 */

static void cali_deinit(void)
{
    if (s_cali) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(s_cali);
#endif
        s_cali = NULL;
    }
}

static bool cali_init(const adc_cfg_t *cfg)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t ccfg = {
        .unit_id  = ADC_UNIT_ID,
        .chan     = (adc_channel_t)cfg->channel,
        .atten    = cfg->atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&ccfg, &s_cali);
    if (err == ESP_OK) {
        return true;
    }
    ESP_LOGW(TAG, "curve fitting 校准初始化失败: %s", esp_err_to_name(err));
    s_cali = NULL;
#else
    (void)cfg;
    ESP_LOGW(TAG, "当前芯片/配置不支持 curve fitting 校准");
#endif
    return false;
}

static int cali_mv(int raw)
{
    int mv = -1;
    if (s_cali) {
        if (adc_cali_raw_to_voltage(s_cali, raw, &mv) != ESP_OK) {
            mv = -1;
        }
    }
    return mv;
}

/* -------------------------------------------------------------- 配置应用 */

static void print_cfg(void)
{
    printf("#CFG,pin=GPIO%d,chan=%d,atten=%d,width=12,n=%d,period_ms=%d,cali=%d\n",
           s_active.channel, s_active.channel, s_active.atten_db,
           s_active.oversample, s_active.period_ms, s_cali ? 1 : 0);
}

static bool apply_pending(void)
{
    adc_cfg_t next = s_pending;
    adc_oneshot_chan_cfg_t cc = {
        .atten    = next.atten,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t err = adc_oneshot_config_channel(s_adc, (adc_channel_t)next.channel, &cc);
    if (err != ESP_OK) {
        printf("#ERR,config_channel,%s\n", esp_err_to_name(err));
        s_pending = s_active;   /* 回滚，避免后续命令带着非法配置重试 */
        return false;
    }

    cali_deinit();
    s_active = next;
    cali_init(&s_active);
    print_cfg();
    return true;
}

/* ------------------------------------------------------------ 整数开方 */

static uint64_t isqrt64(uint64_t x)
{
    uint64_t r = 0;
    uint64_t bit = (uint64_t)1 << 62;
    while (bit > x) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (x >= r + bit) {
            x -= r + bit;
            r = (r >> 1) + bit;
        } else {
            r >>= 1;
        }
        bit >>= 2;
    }
    return r;
}

/* -------------------------------------------------------------- 高速抓取 */

static void burst_capture(int n)
{
    int64_t t0 = esp_timer_get_time();
    printf("#BURST_BEGIN,%d,0,%d,%d\n", n, s_active.channel, s_active.atten_db);

    char buf[512];
    size_t used = 0;
    for (int i = 0; i < n; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, (adc_channel_t)s_active.channel, &raw) != ESP_OK) {
            raw = -1;
        }
        int wrote = snprintf(buf + used, sizeof(buf) - used, "B,%d,%d\n", i, raw);
        if (wrote > 0 && (size_t)wrote < sizeof(buf) - used) {
            used += (size_t)wrote;
        }
        if (used > sizeof(buf) - 32) {
            fwrite(buf, 1, used, stdout);
            used = 0;
        }
    }
    if (used) {
        fwrite(buf, 1, used, stdout);
    }

    int64_t total_us = esp_timer_get_time() - t0;
    printf("#BURST_END,%d,%" PRId64 "\n", n, total_us);
    fflush(stdout);
}


/* ------------------------------------------------------------ Wi-Fi STA/UDP */

static void wifi_print_status(void)
{
    char gw[16] = "0.0.0.0";
    if (s_wifi_gw_ip != 0) {
        esp_ip4_addr_t a = { .addr = s_wifi_gw_ip };
        esp_ip4addr_ntoa(&a, gw, sizeof(gw));
    }
    printf("#WIFI,state=%d,mode=sta,ssid=%s,ip=%s,gw=%s,tx=%" PRIu32 ",rx=%" PRIu32
           ",txb=%" PRIu32 ",rxb=%" PRIu32 "\n",
           s_wifi_on ? 1 : 0, WIFI_SSID, s_wifi_ip, gw,
           s_wifi_tx_pkts, s_wifi_rx_pkts, s_wifi_tx_bytes, s_wifi_rx_bytes);
    fflush(stdout);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    (void)arg;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_got_ip = false;
        s_wifi_gw_ip = 0;
        strcpy(s_wifi_ip, "0.0.0.0");
        if (s_wifi_on) {
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        snprintf(s_wifi_ip, sizeof(s_wifi_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_wifi_gw_ip = ev->ip_info.gw.addr;
        s_wifi_got_ip = true;
        wifi_print_status();
    }
}

static bool wifi_sta_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        printf("#ERR,nvs,%s\n", esp_err_to_name(err));
        return false;
    }

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("#ERR,netif,%s\n", esp_err_to_name(err));
        return false;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        printf("#ERR,event,%s\n", esp_err_to_name(err));
        return false;
    }

    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (s_sta_netif == NULL) {
        printf("#ERR,sta_netif\n");
        return false;
    }

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&ic);
    if (err != ESP_OK) {
        printf("#ERR,wifi_init,%s\n", esp_err_to_name(err));
        return false;
    }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, NULL);

    wifi_config_t wc = {0};
    memcpy(wc.sta.ssid, WIFI_SSID, strlen(WIFI_SSID));
    memcpy(wc.sta.password, WIFI_PASS, strlen(WIFI_PASS));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wc.sta.pmf_cfg.required = false;

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_STA, &wc);
    if (err == ESP_OK) err = esp_wifi_set_ps(WIFI_PS_NONE);
    if (err == ESP_OK) err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);
    if (err != ESP_OK) {
        printf("#ERR,wifi_cfg,%s\n", esp_err_to_name(err));
        return false;
    }
    return true;
}

static int build_dns_query(uint8_t *buf, uint16_t id)
{
    int i = 0;
    buf[i++] = (uint8_t)(id >> 8);
    buf[i++] = (uint8_t)(id & 0xff);
    buf[i++] = 0x01; buf[i++] = 0x00;   /* RD */
    buf[i++] = 0x00; buf[i++] = 0x01;   /* QDCOUNT=1 */
    for (int k = 0; k < 6; k++) buf[i++] = 0;  /* AN/NS/AR */
    const char *labels[2] = { "example", "com" };
    for (int k = 0; k < 2; k++) {
        int len = (int)strlen(labels[k]);
        buf[i++] = (uint8_t)len;
        memcpy(&buf[i], labels[k], (size_t)len);
        i += len;
    }
    buf[i++] = 0x00;
    buf[i++] = 0x00; buf[i++] = 0x01;   /* QTYPE A */
    buf[i++] = 0x00; buf[i++] = 0x01;   /* QCLASS IN */
    return i;
}

static void wifi_traffic_task(void *arg)
{
    (void)arg;
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        printf("#ERR,udp_socket\n");
        s_wifi_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

    struct sockaddr_in bind_addr = {0};
    bind_addr.sin_family = AF_INET;
    bind_addr.sin_port = htons(WIFI_UDP_PORT);
    bind_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        printf("#ERR,udp_bind,%d\n", errno);
        close(sock);
        s_wifi_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    const int payload = 1400;
    char *buf = malloc(payload);
    if (buf == NULL) {
        close(sock);
        s_wifi_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in bcast = {0};
    bcast.sin_family = AF_INET;
    bcast.sin_port = htons(WIFI_UDP_PORT);
    bcast.sin_addr.s_addr = htonl(INADDR_BROADCAST);

    struct sockaddr_in dns = {0};
    dns.sin_family = AF_INET;
    dns.sin_port = htons(53);

    uint32_t seq = 0;
    int64_t last_print = esp_timer_get_time();

    while (s_wifi_on) {
        if (!s_wifi_got_ip) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* 收：主机的包，回显并记住对端 */
        for (int i = 0; i < 32; i++) {
            struct sockaddr_in from;
            socklen_t flen = sizeof(from);
            int n = recvfrom(sock, buf, payload, 0, (struct sockaddr *)&from, &flen);
            if (n <= 0) {
                break;
            }
            s_wifi_rx_pkts++;
            s_wifi_rx_bytes += (uint32_t)n;
            if (from.sin_port != htons(53)) {
                s_wifi_peer_ip = from.sin_addr.s_addr;
                s_wifi_peer_port = from.sin_port;
                int w = sendto(sock, buf, n, 0, (struct sockaddr *)&from, flen);
                if (w > 0) {
                    s_wifi_tx_pkts++;
                    s_wifi_tx_bytes += (uint32_t)w;
                }
            }
        }

        /* 发：优先发给已知主机，否则广播；尽可能压满 Wi-Fi */
        struct sockaddr_in dst = bcast;
        if (s_wifi_peer_ip != 0) {
            dst.sin_addr.s_addr = s_wifi_peer_ip;
            dst.sin_port = s_wifi_peer_port;
        }
        memcpy(buf, &seq, sizeof(seq));
        memset(buf + sizeof(seq), (int)(seq & 0xff), payload - sizeof(seq));
        for (int i = 0; i < 8; i++) {
            int n = sendto(sock, buf, payload, 0, (struct sockaddr *)&dst, sizeof(dst));
            if (n < 0) {
                break;
            }
            s_wifi_tx_pkts++;
            s_wifi_tx_bytes += (uint32_t)n;
        }
        /* DNS 查询网关：即使客户端隔离/无主机，也能持续产生 RX+TX */
        if (s_wifi_gw_ip != 0 && (seq % 50) == 0) {
            uint8_t q[64];
            int qlen = build_dns_query(q, (uint16_t)seq);
            dns.sin_addr.s_addr = s_wifi_gw_ip;
            int n = sendto(sock, q, qlen, 0, (struct sockaddr *)&dns, sizeof(dns));
            if (n > 0) {
                s_wifi_tx_pkts++;
                s_wifi_tx_bytes += (uint32_t)n;
            }
        }
        seq++;

        int64_t now = esp_timer_get_time();
        if (now - last_print > 5000000) {
            wifi_print_status();
            last_print = now;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(buf);
    close(sock);
    s_wifi_task = NULL;
    vTaskDelete(NULL);
}

static void wifi_load_start(void)
{
    if (s_wifi_on) {
        wifi_print_status();
        return;
    }
    if (!s_wifi_inited) {
        if (!wifi_sta_init()) {
            return;
        }
        s_wifi_inited = true;
    }
    s_wifi_on = true;
    esp_err_t err = esp_wifi_start();
    if (err == ESP_ERR_WIFI_STATE) {
        esp_wifi_connect();
    } else if (err != ESP_OK) {
        printf("#ERR,wifi_start,%s\n", esp_err_to_name(err));
        s_wifi_on = false;
        return;
    }
    s_wifi_tx_pkts = s_wifi_rx_pkts = 0;
    s_wifi_tx_bytes = s_wifi_rx_bytes = 0;
    s_wifi_peer_ip = 0;
    s_wifi_peer_port = 0;
    if (s_wifi_task == NULL) {
        xTaskCreate(wifi_traffic_task, "wifi_load", 4096, NULL, 4, &s_wifi_task);
    }
    printf("#WIFI,state=1,mode=sta,ssid=%s,note=connect configured Wi-Fi then run host/wifi_load.py\n",
           WIFI_SSID);
    wifi_print_status();
}

static void wifi_load_stop(void)
{
    if (!s_wifi_on && !s_wifi_inited) {
        wifi_print_status();
        return;
    }
    s_wifi_on = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    if (s_wifi_inited) {
        esp_wifi_disconnect();
        esp_wifi_stop();
    }
    s_wifi_got_ip = false;
    s_wifi_gw_ip = 0;
    strcpy(s_wifi_ip, "0.0.0.0");
    wifi_print_status();
}

/* -------------------------------------------------------------- 命令解析 */

static void cmd_help(void)
{
    printf("#HELP,command,meaning\n");
    printf("#HELP,H,print this help\n");
    printf("#HELP,S,print current config\n");
    printf("#HELP,C <0-4>,select ADC1 channel/GPIO\n");
    printf("#HELP,A <0|2|6|12>,select attenuation in dB\n");
    printf("#HELP,N <1-256>,oversample count\n");
    printf("#HELP,P <10-5000>,sample period in ms\n");
    printf("#HELP,B [n],burst capture n raw samples (default 1024, max 8192)\n");
    printf("#HELP,W <0|1|s>,Wi-Fi AP + UDP flood off/on/status (SSID ADC_Test)\n");
}

static bool parse_int(const char *s, int *out)
{
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s) {
        return false;
    }
    while (*end && isspace((unsigned char)*end)) {
        end++;
    }
    if (*end != '\0') {
        return false;
    }
    *out = (int)v;
    return true;
}

static void handle_command(char *line)
{
    /* 去掉尾部空白 */
    size_t len = strlen(line);
    while (len && isspace((unsigned char)line[len - 1])) {
        line[--len] = '\0';
    }
    char *p = line;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p == '\0') {
        return;
    }

    char cmd = (char)toupper((unsigned char)*p);
    char *arg = p + 1;
    while (*arg && isspace((unsigned char)*arg)) {
        arg++;
    }

    int v = 0;
    switch (cmd) {
    case 'H':
    case '?':
        cmd_help();
        break;
    case 'S':
        s_status_req = true;
        wifi_print_status();
        break;
    case 'C':
        if (!parse_int(arg, &v) || v < ADC_CHAN_MIN || v > ADC_CHAN_MAX) {
            printf("#ERR,channel,expect 0..4\n");
            break;
        }
        s_pending.channel = v;
        s_apply_req = true;
        break;
    case 'A':
        if (!parse_int(arg, &v)) {
            printf("#ERR,atten,expect 0|2|6|12\n");
            break;
        }
        if (v == 2) {
            v = 2; /* 2.5 dB 档按 2 输入 */
        }
        if (v != 0 && v != 2 && v != 6 && v != 12) {
            printf("#ERR,atten,expect 0|2|6|12\n");
            break;
        }
        s_pending.atten_db = v;
        s_pending.atten = (v == 0) ? ADC_ATTEN_DB_0
                        : (v == 2) ? ADC_ATTEN_DB_2_5
                        : (v == 6) ? ADC_ATTEN_DB_6
                                   : ADC_ATTEN_DB_12;
        s_apply_req = true;
        break;
    case 'N':
        if (!parse_int(arg, &v) || v < OVERSAMPLE_MIN || v > OVERSAMPLE_MAX) {
            printf("#ERR,oversample,expect 1..%d\n", OVERSAMPLE_MAX);
            break;
        }
        s_pending.oversample = v;
        s_apply_req = true;
        break;
    case 'P':
        if (!parse_int(arg, &v) || v < PERIOD_MS_MIN || v > PERIOD_MS_MAX) {
            printf("#ERR,period,expect %d..%d\n", PERIOD_MS_MIN, PERIOD_MS_MAX);
            break;
        }
        s_pending.period_ms = v;
        s_apply_req = true;
        break;
    case 'W':
        if (*arg == '\0' || tolower((unsigned char)*arg) == 's') {
            wifi_print_status();
            break;
        }
        if (!parse_int(arg, &v) || (v != 0 && v != 1)) {
            printf("#ERR,wifi,expect 0|1|s\n");
            break;
        }
        if (v) {
            wifi_load_start();
        } else {
            wifi_load_stop();
        }
        break;
    case 'B':
        if (*arg == '\0') {
            v = BURST_DEFAULT;
        } else if (!parse_int(arg, &v) || v < 16 || v > BURST_MAX) {
            printf("#ERR,burst,expect 16..%d\n", BURST_MAX);
            break;
        }
        s_burst_n = v;
        s_burst_req = true;
        break;
    default:
        printf("#ERR,unknown,%c\n", cmd);
        break;
    }
    fflush(stdout);
}

static void cmd_task(void *arg)
{
    (void)arg;
    char line[64];
    size_t len = 0;
    printf("#CMD,ready,send H for help\n");
    fflush(stdout);

    while (1) {
        int c = fgetc(stdin);
        if (c == EOF) {
            /* 主机未连接/USB 重新枚举时读会返回 EOF；清错后继续等，
             * 否则任务会在启动早期直接退出，之后命令永远无响应。 */
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (c == '\r') {
            continue;
        }
        if (c == '\n') {
            line[len] = '\0';
            if (len > 0) {
                handle_command(line);
            }
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = (char)c;
        }
    }
}

/* ------------------------------------------------------------------ 主程序 */

void app_main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_ID,
        .clk_src  = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = s_active.atten,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, (adc_channel_t)s_active.channel, &chan_cfg));
    cali_init(&s_active);

    printf("#FW,adc_linearity,%d\n", FW_VERSION);
    print_cfg();
    printf("#COL,t_ms,raw_x100,raw_min,raw_max,cal_mv,cal_min_mv,cal_max_mv,raw_std_x100\n");

    xTaskCreate(cmd_task, "adc_cmd", 4096, NULL, 3, NULL);

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (s_apply_req) {
            s_apply_req = false;
            apply_pending();
        }
        if (s_status_req) {
            s_status_req = false;
            print_cfg();
        }
        if (s_burst_req) {
            s_burst_req = false;
            burst_capture(s_burst_n);
            last_wake = xTaskGetTickCount();
        }

        int n = s_active.oversample;
        uint32_t sum = 0;
        uint64_t sumsq = 0;
        int raw_min = 4095;
        int raw_max = 0;
        int ok_cnt = 0;

        for (int i = 0; i < n; i++) {
            int raw = 0;
            if (adc_oneshot_read(s_adc, (adc_channel_t)s_active.channel, &raw) == ESP_OK) {
                sum += (uint32_t)raw;
                sumsq += (uint64_t)raw * (uint64_t)raw;
                if (raw < raw_min) raw_min = raw;
                if (raw > raw_max) raw_max = raw;
                ok_cnt++;
            }
        }

        if (ok_cnt == 0) {
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(s_active.period_ms));
            continue;
        }

        uint32_t cnt = (uint32_t)ok_cnt;
        uint32_t raw_x100 = (sum * 100u + cnt / 2u) / cnt;
        int raw_mean = (int)((sum + cnt / 2u) / cnt);
        int64_t t_ms = esp_timer_get_time() / 1000;

        /* σ_x100 = 100 * sqrt(N*Σx² − (Σx)²) / N */
        uint64_t var_num = (uint64_t)cnt * sumsq - (uint64_t)sum * (uint64_t)sum;
        uint32_t std_x100 = (uint32_t)((100u * isqrt64(var_num) + cnt / 2u) / cnt);

        printf("D,%" PRId64 ",%" PRIu32 ",%d,%d,%d,%d,%d,%" PRIu32 "\n",
               t_ms, raw_x100, raw_min, raw_max,
               cali_mv(raw_mean), cali_mv(raw_min), cali_mv(raw_max), std_x100);
        fflush(stdout);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(s_active.period_ms));
    }
}
