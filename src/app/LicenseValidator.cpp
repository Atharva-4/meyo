#include "LicenseValidator.h"

#include <string>
#include <vector>
#include <windows.h>

#include <QCoreApplication>
#include <QString>

typedef char** (*LicValidModules)(const char*, const char*, const char*, const char**, int);

namespace Mayo {

bool LicenseValidator::isLicenseValid()
{
    // lk.dll must be next to Cadify.exe
    const std::string appDir =
        QCoreApplication::applicationDirPath().toStdString();

    const std::string dllPath  = appDir + "/lk.dll";
    const std::string licPath  = "Cadify.lic";   // ← changed from FireSpy.lic
    const std::string cidPath  = "getcid.exe";

    HINSTANCE hDLL = LoadLibraryA(dllPath.c_str());
    if (!hDLL)
        return false;

    auto getValidModules = reinterpret_cast<LicValidModules>(
        GetProcAddress(hDLL, "LicValidModules_C"));

    if (!getValidModules) {
        FreeLibrary(hDLL);
        return false;
    }

    // ← "Cadify" replaces "FireSpy" — must match what's baked into Cadify.lic
    const char* modules[] = {"Cadify", "All", nullptr};

    char** moduleList = getValidModules(
        appDir.c_str(),   // folder where Cadify.lic lives
        licPath.c_str(),  // "Cadify.lic"
        cidPath.c_str(),  // "getcid.exe"
        modules,
        2
    );

    std::vector<std::string> moduleListVec;
    for (int i = 0; moduleList[i] != nullptr; ++i)
        moduleListVec.push_back(moduleList[i]);

    FreeLibrary(hDLL);

    return !moduleListVec.empty();
}

} // namespace Mayo