#pragma once

#include "commands_api.h"

namespace Mayo {

    
    
    class CommandConvert3DXML : public Command {
    public:
        CommandConvert3DXML(IAppContext* context);
        void execute() override;
        static constexpr std::string_view Name = "convert.3dxml";
    };


    

} // namespace Mayo
