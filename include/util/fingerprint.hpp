#ifndef FINGERPRINT_H
#define FINGERPRINT_H

#include <boost/uuid/uuid.hpp>
#include <type_traits>
#include <cstdint>

namespace osrm
{
namespace util
{

// implements a singleton, i.e. there is one and only one conviguration object
class FingerPrint
{
  public:
    static FingerPrint GetValid();
    const boost::uuids::uuid &GetFingerPrint() const;
    bool IsMagicNumberSAME(const FingerPrint &other) const;
    bool IsMajorVersionSAME(const FingerPrint &other) const;
    bool IsMinorVersionSAME(const FingerPrint &other) const;
    bool IsPatchVersionSAME(const FingerPrint &other) const;
    bool IsChecksumValid() const;

    int GetMajorVersion() const;
    int GetMinorVersion() const;
    int GetPatchVersion() const;

  private:
    std::uint32_t magic_number;
    char major_version;
    char minor_version;
    char patch_version;
    char crc8;
};

static_assert(sizeof(FingerPrint) == 8, "FingerPrint has unexpected size");
static_assert(std::is_trivial<FingerPrint>::value, "FingerPrint needs to be trivial.");
}
}

#endif /* FingerPrint_H */
