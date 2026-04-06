#pragma once

#include "commands_api.h"
#include "../gui/gui_application.h"

namespace Mayo {

    // ─────────────────────────────────────────────────────────────────────────────
    //  Shown under the "Convertor" menu tab → single "Convertor" action
    //  - Reads the currently open document automatically
    //  - Shows a dialog: which format to convert to
    //  - Asks where to save via QFileDialog
    //  - Runs conversion on a background thread
    // ─────────────────────────────────────────────────────────────────────────────
    class CommandConvertor : public Command
    {
        Q_OBJECT
    public:
        explicit CommandConvertor(IAppContext* context);
        void execute() override;

        static constexpr std::string_view Name = "convertor";

    private:
        bool m_isRunning = false;
    };

} // namespace Mayo
