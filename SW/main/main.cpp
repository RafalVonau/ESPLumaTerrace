/**
 * The program is designed to control LED lights on a terrace.
 * It operates by connecting to a Node.js server, and if a connection is unavailable, it attempts to synchronize the time with an NTP server.
 * Afterward, it calculates the time of sunset to manage the lighting accordingly.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <map>
#include <set>
#include <string>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/semphr.h"
#include "sdkconfig.h"
#include <esp_event.h>
#include <esp_event_loop.h>
#include <string>
#include <esp_task_wdt.h>
#include <freertos/task.h>
#include "arduino.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_wifi.h"
#include "driver/rtc_io.h"
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h> // struct addrinfo
#include <arpa/inet.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"
#include "esp_pm.h"

/* Configurable parameters: */
#define WIFI_SSID "XXXXX"
#define WIFI_PASS "XXXXX"

const double latitude = 51.101341168974734;
const double longitude = 16.94544991194574;

void setup_timezone(void)
{
    /* Europe/Warsaw */
    setenv("TZ", "CET-1CEST,M3.5.0/02:00:00,M10.5.0/03:00:00", 1);
    tzset();
}

// NTP server
#define NTP_PACKET_SIZE 48
#define NTP_PORT 123
#define NTP_SERVER_IP "91.206.8.36"

/* ========================================================================================== */

#ifdef CONFIG_IDF_TARGET_ESP32C3
#define LED_GPIO GPIO_NUM_1
#define BUTTON_GPIO GPIO_NUM_0
#else
#define LED_GPIO GPIO_NUM_4
#define BUTTON_GPIO GPIO_NUM_13
#endif

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 30       /* Time ESP32 will go to sleep (in seconds) */
#define TIME_LED_ACTIVE (3600 * 4)

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int update_alarm = 0;
RTC_DATA_ATTR int gled = 0;
RTC_DATA_ATTR int factor = 8;
RTC_DATA_ATTR int time_synced = 0;

// Sunrise
#define DEFAULT_ZENITH 90.8333
#define DEGREES_PER_HOUR (360.0 / 24.0)
#define MSEC_IN_HOUR (60 * 60 * 1000)

/* Global variables */
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static char tag[] = "terrace";

#ifdef DEBUG
#define msg_info(fmt, args...) ESP_LOGI(tag, fmt, ##args);
#define msg_init(fmt, args...) ESP_LOGI(tag, fmt, ##args);
#define msg_debug(fmt, args...) ESP_LOGI(tag, fmt, ##args);
#define msg_error(fmt, args...) ESP_LOGE(tag, fmt, ##args);
#else
#define msg_info(fmt, args...)
#define msg_init(fmt, args...)
#define msg_debug(fmt, args...)
#define msg_error(fmt, args...) ESP_LOGE(tag, fmt, ##args);
#endif

/**
 * Method to print the reason by which ESP32
 * has been awaken from sleep
 */
void print_wakeup_reason()
{
    esp_sleep_wakeup_cause_t wakeup_reason;
    wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason)
    {
    case ESP_SLEEP_WAKEUP_GPIO: // 05 - this is used by ESP32-C3
        msg_init("Wakeup by GPIO");
        update_alarm = 1;
        break;
    case ESP_SLEEP_WAKEUP_EXT0:
        msg_debug("Wakeup caused by external signal using RTC_IO");
        update_alarm = 1;
        break;
    case ESP_SLEEP_WAKEUP_EXT1:
        msg_debug("Wakeup caused by external signal using RTC_CNTL");
        break;
    case ESP_SLEEP_WAKEUP_TIMER:
        msg_debug("Wakeup caused by timer");
        break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
        msg_debug("Wakeup caused by touchpad");
        break;
    case ESP_SLEEP_WAKEUP_ULP:
        msg_debug("Wakeup caused by ULP program");
        break;
    default:
        msg_debug("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
        break;
    }
}

/**
 * \brief Prepare io pins.
 */
void rtc_prepare_pins(void)
{
    digitalWrite(LED_GPIO, gled ? HIGH : LOW);
#ifdef CONFIG_IDF_TARGET_ESP32C3
    gpio_hold_en(LED_GPIO);
    gpio_deep_sleep_hold_en();
    gpio_hold_en(LED_GPIO);
#else
    rtc_gpio_isolate(GPIO_NUM_12);
    gpio_hold_en(LED_GPIO);
#endif
}

/**
 * \brief Restore io pins.
 */
void rtc_restore_pins(void)
{
#ifdef CONFIG_IDF_TARGET_ESP32C3
    gpio_hold_dis(LED_GPIO);
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(LED_GPIO);
#else
    gpio_deep_sleep_hold_dis();
    gpio_hold_dis(LED_GPIO);
#endif
    gpio_reset_pin(LED_GPIO);
    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, gled ? HIGH : LOW);
    pinMode(BUTTON_GPIO, INPUT_PULLUP);
}

/**
 * Detect NTP wrap for year 2036 (it is working to 2136).
 */
uint64_t correct_ntp_wrap(uint32_t secsSince1900)
{
    const uint64_t SECONDS_IN_100_YEARS = 3155673600ULL;  // 100 years in seconds
    const uint64_t WRAPAROUND_CORRECTION = 4294967296ULL; // 2^32

    // Convert to 64-bit for calculation
    uint64_t corrected_time = (uint64_t)secsSince1900;

    // Check if the timestamp is less than 100 years from 1900
    if (corrected_time < SECONDS_IN_100_YEARS)
    {
        corrected_time += WRAPAROUND_CORRECTION; // Correct for wrap-around
    }
    return corrected_time;
}

/**
 * \brief Synchronize time from NTP server.
 */
bool syncTime(int timeout_ms)
{
    int sock;
    struct sockaddr_in serverAddr;
    uint8_t packetBuffer[NTP_PACKET_SIZE];

    // Initialize the NTP packet with zeroes and then set up the required fields
    memset(packetBuffer, 0, NTP_PACKET_SIZE);
    packetBuffer[0] = 0b11100011; // LI, Version, Mode
    packetBuffer[1] = 0;          // Stratum, or type of clock
    packetBuffer[2] = 6;          // Polling Interval
    packetBuffer[3] = 0xEC;       // Peer Clock Precision
    // Root Delay & Root Dispersion (set to zero)
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;

    // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        msg_error("Unable to create socket: errno %d", errno);
        return false;
    }

    // Set server address
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(NTP_PORT);
    inet_pton(AF_INET, NTP_SERVER_IP, &serverAddr.sin_addr.s_addr);

    // Send the NTP packet to the server
    if (sendto(sock, packetBuffer, NTP_PACKET_SIZE, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        msg_error("Error sending packet: errno %d", errno);
        close(sock);
        return false;
    }

    // Set socket receive timeout
    struct timeval timeout;
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Receive response
    struct sockaddr_in sourceAddr;
    socklen_t addrLen = sizeof(sourceAddr);
    int recvLen = recvfrom(sock, packetBuffer, NTP_PACKET_SIZE, 0, (struct sockaddr *)&sourceAddr, &addrLen);

    if (recvLen < 0)
    {
        msg_error("Receive failed: errno %d", errno);
        close(sock);
        return false;
    }
    // Check if we received the correct amount of data
    if (recvLen >= NTP_PACKET_SIZE)
    {
        // Extract the timestamp (the time since 1900) from the received NTP packet
        uint32_t secsSince1900 = (packetBuffer[40] << 24) | (packetBuffer[41] << 16) |
                                 (packetBuffer[42] << 8) | packetBuffer[43];

        // Convert NTP time to UNIX time (seconds since 1970)
        /* Year 2036 safe code */
        const time_t SEVENTY_YEARS = 2208988800UL;
        time_t unixTime = correct_ntp_wrap(secsSince1900) - SEVENTY_YEARS;
        // Set system time
        struct timeval tv;
        tv.tv_sec = unixTime;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);

        msg_info("Time synchronized: %ld", tv.tv_sec);

        close(sock);
        time_synced = 1;
        return true;
    }
    msg_error("Invalid NTP response");
    close(sock);
    return false;
}

/**
 * \brief Calculate timestamp at 1:00 AM.
 */
time_t get_time_1am(void)
{
    time_t now;
    struct tm timeinfo;

    // Get the current time
    time(&now);
    localtime_r(&now, &timeinfo); // Get local time based on timezone set

    // Create a struct tm for 1:00 AM of the same day
    struct tm time_1am = timeinfo;
    time_1am.tm_hour = 1;
    time_1am.tm_min = 0;
    time_1am.tm_sec = 0;
    time_t target_time = mktime(&time_1am);
    gmtime_r(&target_time, &timeinfo); // UTC time (no timezone offset)
    msg_debug("1AM UTC hour = %d", timeinfo.tm_hour);
    /* if timestamp crosses hour 00:00 add one day more */
    if (timeinfo.tm_hour == 0)
    {
        time_1am.tm_mday += 1; // Move to the next day
        target_time = mktime(&time_1am);
    }
    return target_time;
}

/**
 * Get day of year
 */
int getDayOfYear(struct tm *date)
{
    struct tm start_of_year = {
        .tm_sec = 0,
        .tm_min = 0,
        .tm_hour = 0,
        .tm_mday = 1,
        .tm_mon = 0,
        .tm_year = date->tm_year,
        .tm_wday = 0,
        .tm_yday = 0,
        .tm_isdst = 0};
    time_t start_time = mktime(&start_of_year);
    time_t current_time = mktime(date);
    return (int)difftime(current_time, start_time) / 86400 + 1;
}

/**
 * Helper trigonometric functions working with degrees
 */
double sinDeg(double deg) { return sin(deg * M_PI / 180.0); }
double acosDeg(double x) { return acos(x) * 180.0 / M_PI; }
double asinDeg(double x) { return asin(x) * 180.0 / M_PI; }
double tanDeg(double deg) { return tan(deg * M_PI / 180.0); }
double cosDeg(double deg) { return cos(deg * M_PI / 180.0); }
double mod(double a, double b)
{
    double result = fmod(a, b);
    return result < 0 ? result + b : result;
}

/**
 * Calculate Date for either sunrise or sunset
 */
time_t calculate(double latitude, double longitude, int isSunrise, double zenith, struct tm *date)
{
    int dayOfYear = getDayOfYear(date);
    double hoursFromMeridian = longitude / DEGREES_PER_HOUR;

    double approxTimeOfEventInDays = isSunrise ? dayOfYear + ((6.0 - hoursFromMeridian) / 24.0) : dayOfYear + ((18.0 - hoursFromMeridian) / 24.0);

    double sunMeanAnomaly = (0.9856 * approxTimeOfEventInDays) - 3.289;
    double sunTrueLongitude = mod(sunMeanAnomaly + (1.916 * sinDeg(sunMeanAnomaly)) + (0.020 * sinDeg(2 * sunMeanAnomaly)) + 282.634, 360.0);

    double ascension = 0.91764 * tanDeg(sunTrueLongitude);
    double rightAscension = mod(atan(ascension) * 180.0 / M_PI, 360.0);

    int lQuadrant = ((int)(sunTrueLongitude / 90)) * 90;
    int raQuadrant = ((int)(rightAscension / 90)) * 90;
    rightAscension = rightAscension + (lQuadrant - raQuadrant);
    rightAscension /= DEGREES_PER_HOUR;

    double sinDec = 0.39782 * sinDeg(sunTrueLongitude);
    double cosDec = cosDeg(asinDeg(sinDec));

    double cosLocalHourAngle = (cosDeg(zenith) - (sinDec * sinDeg(latitude))) / (cosDec * cosDeg(latitude));

    double localHourAngle = isSunrise ? 360.0 - acosDeg(cosLocalHourAngle) : acosDeg(cosLocalHourAngle);
    double localHour = localHourAngle / DEGREES_PER_HOUR;

    double localMeanTime = localHour + rightAscension - (0.06571 * approxTimeOfEventInDays) - 6.622;
    double time = mod(localMeanTime - hoursFromMeridian, 24.0);

    struct tm utc_midnight = *date;
    utc_midnight.tm_hour = 0;
    utc_midnight.tm_min = 0;
    utc_midnight.tm_sec = 0;

    return mktime(&utc_midnight) + (time * 3600);
}

/**
 * Calculate Sunrise time for given longitude, latitude, zenith and date
 */
time_t getSunrise(double latitude, double longitude, struct tm *date)
{
    return calculate(latitude, longitude, 1, DEFAULT_ZENITH, date);
}

/**
 * Calculate Sunset time for given longitude, latitude, zenith and date
 */
time_t getSunset(double latitude, double longitude, struct tm *date)
{
    return calculate(latitude, longitude, 0, DEFAULT_ZENITH, date);
}

/**
 *\brief Try to controll without the server.
 */
uint64_t calcTimeToSleep(bool wifiFail)
{
    /* Try to synchronize time */
    if (!wifiFail)
    {
        if (!syncTime(2000))
        {
            syncTime(4000);
        }
    }
    if ((!time_synced) && (update_alarm))
    {
        /* Try to sync time besed on button press - use dummy date  */
        time_t unixTime = 1729863490ul + 2ul * 3600ul;
        if (gled)
        {
            unixTime = 1729892290ul + 2ul * 3600ul;
        }
        struct timeval tv;
        tv.tv_sec = unixTime;
        tv.tv_usec = 0;
        settimeofday(&tv, NULL);
        msg_error("Set dummy date");
    }
    // Get current time from the ESP32 system clock
    struct timeval now;
    gettimeofday(&now, NULL);
    // Convert to `struct tm` for easier manipulation
    struct tm *date = localtime(&now.tv_sec);
    time_t sunset = getSunset(latitude, longitude, date);
    setup_timezone();
    time_t time_1am = get_time_1am();
    time_t tmp = now.tv_sec;

    msg_info("Now (UTC): %s", asctime(gmtime(&tmp)));
    msg_info("Sunset (UTC): %s", asctime(gmtime(&sunset)));
    msg_info("1AM (UTC): %s", asctime(gmtime(&time_1am)));

    // msg_info("Now (local): %s", asctime(localtime(&tmp)));
    // msg_info("Sunset (local): %s", asctime(localtime(&sunset)));
    // msg_info("1AM (local): %s", asctime(localtime(&time_1am)));

    if (update_alarm)
    {
        /* Handle button press - manual ON/OFF */
        if (gled)
        {
            gled = 0;
            while (sunset <= tmp)
                sunset += (24u * 3600u);
            tmp = sunset - tmp;
            msg_info("BUTTON LED OFF for %" PRIu64 " [h]", tmp / 3600);
            digitalWrite(LED_GPIO, gled ? HIGH : LOW);
            return tmp;
        }
        else
        {
            gled = 1;
            while (time_1am <= tmp)
                time_1am += (24u * 3600u);
            tmp = time_1am - tmp;
            msg_info("BUTTON LED ON for %" PRIu64 " [h]", tmp / 3600);
            digitalWrite(LED_GPIO, gled ? HIGH : LOW);
            return tmp;
        }
    }
    else if ((tmp >= sunset) && (tmp < time_1am))
    {
        gled = 1;
        // Calculate the seconds until 1:00 AM
        // tmp = difftime(time_1am, tmp);
        tmp = time_1am - tmp;
        msg_info("LED ON for %" PRIu64 " [h]", tmp / 3600);
        digitalWrite(LED_GPIO, gled ? HIGH : LOW);
        return tmp;
    }
    else
    {
        gled = 0;
        // tmp = difftime(sunset, tmp);
        while (sunset <= tmp)
            sunset += (24u * 3600u);
        tmp = sunset - tmp;
        msg_info("LED OFF for %" PRIu64 " [h]", tmp / 3600);
        digitalWrite(LED_GPIO, gled ? HIGH : LOW);
        return tmp;
    }

    return 5;
}

/**
 * \brief Enter deep sleep mode.
 * \param sleepTimeSeconds - seconds to sleep to.
 */
void goToDeepSleep(bool wifiFail = false)
{
    /* Use fallback algorithm */
    uint64_t sleepTimeSeconds = calcTimeToSleep(wifiFail);

    // disconnect WiFi as it's no longer needed
    esp_wifi_stop();
    // do noot sleep shorter than 5s.
    if (sleepTimeSeconds < 5)
        sleepTimeSeconds = 5;
    msg_debug("Going to sleep now for %" PRIu64 " [s]\n", sleepTimeSeconds);
    // Change seconds to microseconds.
    sleepTimeSeconds *= uS_TO_S_FACTOR;
    // esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    /* GO TO SLEEP */
    esp_sleep_enable_timer_wakeup(sleepTimeSeconds);
#ifdef CONFIG_IDF_TARGET_ESP32C3
    esp_deep_sleep_enable_gpio_wakeup(1u << BUTTON_GPIO, ESP_GPIO_WAKEUP_GPIO_LOW);
#else
    esp_sleep_enable_ext0_wakeup(BUTTON_GPIO, 0);
#endif
    rtc_prepare_pins();
    esp_deep_sleep_start();
}

/**
 * \brief WiFi event handler.
 */
static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        /* Upss: WiFi connection fail :-( */
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        msg_info("got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/*!
 * \brief Connect to WiFi network.
 */
bool connectToWiFi()
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    // Connect to WiFi
    ESP_ERROR_CHECK(ret);
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS((factor > 7) ? 3000 : 5000));

    return (bits & WIFI_CONNECTED_BIT);
}

/**
 * \brief Standard connect but with timeout.
 */
int connect_timeout(int sock, const struct sockaddr *addr, socklen_t addrlen, int timeout_sec)
{
    // Set the socket to non-blocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        return -1; // Failed to set non-blocking
    }

    // Start the connection attempt
    int err = connect(sock, addr, addrlen);
    if (err < 0 && errno != EINPROGRESS)
    {
        return -1; // Connection failed immediately
    }

    // Use select() to wait for the connection or timeout
    fd_set set;
    struct timeval timeout;
    timeout.tv_sec = timeout_sec;
    timeout.tv_usec = 0;

    FD_ZERO(&set);
    FD_SET(sock, &set);

    err = select(sock + 1, NULL, &set, NULL, &timeout);
    if (err == 0)
    {
        errno = ETIMEDOUT; // Connection timed out
        return -1;
    }
    else if (err < 0)
    {
        return -1; // Error during select()
    }

    // Check for connection success or error using getsockopt
    int so_error;
    socklen_t len = sizeof(so_error);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
    if (so_error != 0)
    {
        errno = so_error; // Propagate socket error
        return -1;
    }

    // Restore socket to blocking mode
    if (fcntl(sock, F_SETFL, flags) < 0)
    {
        return -1; // Failed to restore blocking mode
    }
    return 0; // Success
}

/**
 * MAIN
 */
extern "C" void app_main(void)
{
    esp_err_t ret;
    // Increment boot number and print it every reboot
    // vTaskDelay(pdMS_TO_TICKS(5000));
    ++bootCount;
    msg_init("Boot number: %d", bootCount);
    rtc_restore_pins();
    print_wakeup_reason();
#if 1
    /* Configure light sleep */
    esp_pm_config_t m_pcfg;
    m_pcfg.light_sleep_enable = true;
    m_pcfg.max_freq_mhz = 80;
    m_pcfg.min_freq_mhz = 80;
    ret = esp_pm_configure(&m_pcfg);
    if (ret != ESP_OK)
    {
        msg_error("Can not configure light sleep mode :-(  (ret = %d)", ret);
    }
#endif
    if (connectToWiFi())
    {
        goToDeepSleep();
    }
    msg_error("Unable to connect to WiFi!");
    goToDeepSleep(true);
    vTaskDelete(NULL);
}
