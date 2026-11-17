// Experimental Plantronics BT600 battery polling.
// Protocol reference: Sapd/HeadsetControl lib/devices/plantronics_bt600.hpp.
// That implementation is verified for the Voyager 8200 UC; compatibility with
// the Voyager 5200 is NOT yet confirmed. This driver only reads battery state.
#include "drivers.h"

namespace {

constexpr USHORT kPlantronics = 0x047F;
constexpr BYTE kReportId = 0x07;
constexpr BYTE kHdr1 = 0x01, kHdr2 = 0x01, kHdr3 = 0x10;
constexpr BYTE kDongle = 0x00, kHeadset = 0x20, kHeadsetReply = 0x02;
constexpr BYTE kHello = 0x01, kGet = 0x02, kHelloAck = 0x08, kGetReply = 0x03;
constexpr BYTE kBatteryGroup = 0x0A, kBatteryItem = 0x1A;

std::vector<BYTE> Frame(BYTE op, BYTE group, BYTE item, BYTE address) {
    std::vector<BYTE> f(64, 0);
    f[0] = kReportId;
    f[1] = kHdr1; f[2] = kHdr2; f[3] = kHdr3;
    f[4] = op == kGet ? 0x06 : 0x07;
    f[5] = address;
    f[8] = op; f[9] = group; f[10] = item;
    return f;
}

bool IsReply(const std::vector<BYTE>& r, BYTE op, BYTE group, BYTE item, BYTE address) {
    return r.size() >= 17 && r[0] == kReportId && r[1] == kHdr1 &&
           r[2] == kHdr2 && r[3] == kHdr3 && r[5] == address &&
           r[8] == op && r[9] == group && r[10] == item;
}

// The BT600 vendor collection has no interrupt-OUT endpoint. SetOutputAndRead
// uses HidD_SetOutputReport (EP0) and arms the input read before sending.
bool SendAndWait(HidDev& d, const std::vector<BYTE>& request, BYTE op,
                 BYTE group, BYTE item, BYTE address, std::vector<BYTE>& reply,
                 int timeoutMs) {
    int n = d.SetOutputAndRead(request, reply, timeoutMs);
    if (n > 0 && IsReply(reply, op, group, item, address)) return true;

    ULONGLONG end = GetTickCount64() + timeoutMs;
    for (int attempt = 0; attempt < 15; ++attempt) {
        int left = (int)((LONGLONG)end - (LONGLONG)GetTickCount64());
        if (left <= 0) break;
        n = d.Read(reply, left);
        if (n < 0) break;
        if (IsReply(reply, op, group, item, address)) return true;
    }
    return false;
}

bool OpenSession(HidDev& d) {
    // Poly's Windows attach sequence starts with this two-byte output report.
    // It is a session initialisation nudge, not a setting change.
    std::vector<BYTE> ignored;
    d.SetOutputAndRead({ 0x06, 0x01 }, ignored, 80);

    std::vector<BYTE> reply;
    auto dongleHello = Frame(kHello, 0x01, 0x02, kDongle);
    if (!SendAndWait(d, dongleHello, kHelloAck, 0x01, 0x02, kDongle, reply, 350)) return false;

    auto headsetHello = Frame(kHello, 0x01, 0x02, kHeadset);
    return SendAndWait(d, headsetHello, kHelloAck, 0x01, 0x02, kHeadsetReply, reply, 500);
}

bool ReadBattery(HidDev& d, int& percent, bool& charging) {
    if (!OpenSession(d)) return false;
    d.Flush();
    auto get = Frame(kGet, kBatteryGroup, kBatteryItem, kHeadset);
    std::vector<BYTE> r;
    if (!SendAndWait(d, get, kGetReply, kBatteryGroup, kBatteryItem, kHeadsetReply, r, 700)) return false;

    BYTE level = r[11], count = r[12], charge = r[13];
    if (count < 2 || level >= count || charge > 1) return false;
    percent = (int)level * 100 / ((int)count - 1);
    charging = charge != 0;
    return percent >= 0 && percent <= 100;
}

}  // namespace

void PollPlantronics(PollContext& ctx) {
    // Both USB product IDs are reported for the BT600 adapter family.
    for (USHORT pid : { (USHORT)0x02F7, (USHORT)0x02EE }) {
        if (ctx.IsClaimed(kPlantronics, pid)) continue;
        for (const HidColl* c : ctx.Find(kPlantronics, pid, [](const HidColl& c) {
                 return c.iface == 3 && c.page() == 0xFFA2 && c.usage() == 0x0003 &&
                        c.outLen() >= 64 && c.inLen() >= 64;
             })) {
            HidDev d;
            if (!d.Open(*c)) continue;
            int percent = -1;
            bool charging = false;
            if (!ReadBattery(d, percent, charging)) continue;

            DeviceStatus s;
            s.key = L"plantronics:bt600:" + Hex4(pid);
            s.name = L"Plantronics headset (BT600)";
            s.via = L"Plantronics BT600";
            s.kind = DeviceKind::Headset;
            s.percent = percent;
            s.charging = charging;
            ctx.Add(s, kPlantronics, pid);
            break;
        }
    }
}
