#include "util/fingerprint.hpp"
#include "util/exception.hpp"
#include "util/exception_utils.hpp"
#include "util/version.hpp"

#include <boost/uuid/name_generator.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <cstring>

#include <algorithm>
#include <string>

namespace osrm
{
namespace util
{
namespace // anonymous
{

std::vector<std::uint8_t> CalulateTable_CRC8()
{
    const std::uint8_t generator = 0x1D;

    std::vector<std::uint8_t> crctable(256);
    /* iterate over all byte values 0 - 255 */
    for (int divident = 0; divident < 256; divident++)
    {
        std::uint8_t currByte = static_cast<std::uint8_t>(divident);
        /* calculate the CRC-8 value for current byte */
        for (std::uint8_t bit = 0; bit < 8; bit++)
        {
            if ((currByte & 0x80) != 0)
            {
                currByte <<= 1;
                currByte ^= generator;
            }
            else
            {
                currByte <<= 1;
            }
        }
        /* store CRC value in lookup table */
        crctable[divident] = currByte;
    }
    return crctable;
}

std::uint8_t Compute_CRC8(const std::uint8_t bytes[], const std::size_t count)
{
    auto crctable = CalulateTable_CRC8();
    std::uint8_t crc = 0;
    for (std::size_t i = 0; i < count; i++)
    {
        /* XOR-in next input byte */
        auto data = bytes[i] ^ crc;
        /* get current CRC value = remainder */
        crc = crctable[data];
    }

    return crc;
}
}

FingerPrint FingerPrint::GetValid()
{
    FingerPrint fingerprint;

    // 4 chars, 'O','S','R','M'
    fingerprint.magic_number = 1297240911;
    fingerprint.major_version = static_cast<char>(atoi(OSRM_VERSION_MAJOR));
    fingerprint.minor_version = static_cast<char>(atoi(OSRM_VERSION_MINOR));
    fingerprint.patch_version = static_cast<char>(atoi(OSRM_VERSION_PATCH));
    fingerprint.crc8 = Compute_CRC8(reinterpret_cast<const std::uint8_t *>(&fingerprint), 7);

    return fingerprint;
}

int FingerPrint::GetMajorVersion() const { return major_version; }
int FingerPrint::GetMinorVersion() const { return minor_version; }
int FingerPrint::GetPatchVersion() const { return patch_version; }

bool FingerPrint::IsMagicNumberSAME(const FingerPrint &other) const
{
    return other.magic_number == magic_number;
}
bool FingerPrint::IsMajorVersionSAME(const FingerPrint &other) const
{
    return IsMagicNumberSAME(other) && other.major_version == major_version;
}
bool FingerPrint::IsMinorVersionSAME(const FingerPrint &other) const
{
    return IsMajorVersionSAME(other) && other.minor_version == minor_version;
}
bool FingerPrint::IsPatchVersionSAME(const FingerPrint &other) const
{
    return IsMinorVersionSAME(other) && other.patch_version == patch_version;
}

bool FingerPrint::IsChecksumValid() const
{
    return crc8 == Compute_CRC8(reinterpret_cast<const std::uint8_t *>(this), 7);
}
}
}
