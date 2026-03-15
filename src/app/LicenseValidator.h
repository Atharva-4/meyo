#pragma once

namespace Mayo {

class LicenseValidator
{
public:
    // Returns true if Cadify.lic is valid for this machine
    static bool isLicenseValid();
};

} // namespace Mayo