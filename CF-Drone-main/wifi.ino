// Wi-Fi功能支持
//
// ==================== 这个文件是干什么的 ====================
// 管理飞控的 WiFi 连接，支持三种模式：
//   W_DISABLED = 0  完全关闭（最省资源，CPU 全留给飞控）
//   W_AP       = 1  飞控自己当热点（默认，SSID=Drone_WiFi，密码 12345678）
//   W_STA      = 2  飞控去连你家路由器（方便手机同时上网）
// 模式和 SSID/密码都能用 CLI 在线修改（wifi / ap / sta 命令），存 Flash 掉电不丢。
//
// ==================== 谁在用这些网络能力 ====================
//   • web_rc.ino  → 网页遥控器（HTTP，端口 80）
//   • mavlink.ino → MAVLink 遥测（UDP，默认端口 14550）
//
// ==================== 两个必须知道的关键点 ====================
// 1) WiFi.setSleep(false) 必须在开启 AP / 连接之前调用。
//    它关闭 WiFi 省电休眠。若开着休眠，WiFi 芯片会周期性打盹，
//    导致控包延迟忽大忽小 —— 对飞控而言这是致命的。
//
// 2) UDP 是"无连接"的，飞控事先并不知道地面站地址。
//    所以策略是：收到谁发来的包，就把 udpRemoteIP 记成谁，
//    之后所有遥测数据都发回这个地址（见 receiveWiFi / sendWiFi）。

#if WIFI_ENABLED

#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiUdp.h>
#include "Preferences.h"

extern Preferences storage; // 复用主存储

const int W_DISABLED = 0, W_AP = 1, W_STA = 2;
int wifiMode = W_AP;
int udpLocalPort = 14550;
int udpRemotePort = 14550;
IPAddress udpRemoteIP = "255.255.255.255";

WiFiUDP udp;

void setupWiFi() {
	print("Setup Wi-Fi\n");
	WiFi.setSleep(false); // disable power save - must be set before softAP/begin
	if (wifiMode == W_AP) {
		WiFi.softAP(
			storage.getString("WIFI_AP_SSID", "Drone_WiFi").c_str(),
			storage.getString("WIFI_AP_PASS", "12345678").c_str()
		);
	} else if (wifiMode == W_STA) {
		WiFi.begin(
			storage.getString("WIFI_STA_SSID", "").c_str(),
			storage.getString("WIFI_STA_PASS", "").c_str()
		);
	}
	udp.begin(udpLocalPort);
}

void sendWiFi(const uint8_t *buf, int len) {
	if (WiFi.softAPgetStationNum() == 0 && !WiFi.isConnected()) return;
	udp.beginPacket(udpRemoteIP, udpRemotePort);
	udp.write(buf, len);
	udp.endPacket();
}

int receiveWiFi(uint8_t *buf, int len) {
	udp.parsePacket();
	if (udp.remoteIP()) udpRemoteIP = udp.remoteIP();
	return udp.read(buf, len);
}

void printWiFiInfo() {
	if (WiFi.getMode() == WIFI_MODE_AP) {
		print("Mode: Access Point (AP)\n");
		print("MAC: %s\n", WiFi.softAPmacAddress().c_str());
		print("SSID: %s\n", WiFi.softAPSSID().c_str());
		print("Password: ***\n");
		print("Clients: %d\n", WiFi.softAPgetStationNum());
		print("IP: %s\n", WiFi.softAPIP().toString().c_str());
	} else if (WiFi.getMode() == WIFI_MODE_STA) {
		print("Mode: Client (STA)\n");
		print("Connected: %d\n", WiFi.isConnected());
		print("MAC: %s\n", WiFi.macAddress().c_str());
		print("SSID: %s\n", WiFi.SSID().c_str());
		print("Password: ***\n");
		print("IP: %s\n", WiFi.localIP().toString().c_str());
	} else {
		print("Mode: Disabled\n");
		return;
	}
	print("Remote IP: %s\n", udpRemoteIP.toString().c_str());
	print("MAVLink connected: %d\n", mavlinkConnected);
}

void configWiFi(bool ap, const char *ssid, const char *password) {
	if (ap) {
		storage.putString("WIFI_AP_SSID", ssid);
		storage.putString("WIFI_AP_PASS", password);
	} else {
		storage.putString("WIFI_STA_SSID", ssid);
		storage.putString("WIFI_STA_PASS", password);
	}
	print("✓ 重启后生效 Reboot to apply new settings\n");
}

#endif
