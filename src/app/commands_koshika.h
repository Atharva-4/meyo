#pragma once

#include "commands_api.h"

namespace Mayo {

    class CommandCutting : public Command {
    public:
        enum class CutPlane {
            X,
            Y,
            Z
        };

        CommandCutting(IAppContext* context);

        void execute() override;

        static constexpr std::string_view Name = "cutting";

    private:
        bool m_isRunning = false;
    };

    class CommandMergeSTL : public Command
    {
    public:
        CommandMergeSTL(IAppContext* context);
        void execute() override;

        static constexpr std::string_view Name = "merge.stl";
    };

    // New: Hole filling commands
    class CommandHoleFillingFull : public Command {
    public:
        CommandHoleFillingFull(IAppContext* context);
        void execute() override;
        static constexpr std::string_view Name = "holefilling.full";

    private:
        bool m_isRunning = false;  
    };


    class CommandHoleFillingSelected : public Command {
    public:
        CommandHoleFillingSelected(IAppContext* context);
        void execute() override;
        static constexpr std::string_view Name = "holefilling.selected";

    private:
        bool m_isRunning = false;
    };


    // Point to Surface - With Normals (needs CUDA)
    class CommandPointToSurfaceWithNormals : public Command {
    public:
        CommandPointToSurfaceWithNormals(IAppContext* context);
        void execute() override;
        static constexpr std::string_view Name = "pointtosurface.withnormals";
    };

    // Point to Surface - Without Normals (no CUDA needed)
    class CommandPointToSurfaceWithoutNormals : public Command {
    public:
        CommandPointToSurfaceWithoutNormals(IAppContext* context);
        void execute() override;
        static constexpr std::string_view Name = "pointtosurface.withoutnormals";
    };

    class CommandConvert3DXML : public Command {
    public:
        CommandConvert3DXML(IAppContext* context);
        void execute() override;
        static constexpr std::string_view Name = "convert.3dxml";
    };

    class CommandSimplification : public Command
    {
        Q_OBJECT
    public:
        static constexpr const char Name[] = "simplify-mesh";

        explicit CommandSimplification(IAppContext* context);
        void execute() override;

    private:
        bool m_isRunning = false;
    };
    
    class CommandHollowing : public Command {
        Q_OBJECT
    public:
        static constexpr const char Name[] = "hollow-mesh";
        explicit CommandHollowing(IAppContext* context);
        void execute() override;
        bool getEnabledStatus() const override { return true; }

    private:
        bool m_isRunning = false;
    };


    class CommandMeshRepairStatistics : public Command {
    public:
        CommandMeshRepairStatistics(IAppContext* context);
        void execute() override;
        static constexpr std::string_view Name = "meshrepair.statistics";

    private:
        bool m_isRunning = false;
    };

    class CommandMeshAutoRepair : public Command {
    public:
        CommandMeshAutoRepair(IAppContext* context);
        void execute() override;

        static constexpr std::string_view Name = "meshrepair.autorepair";

    private:
        bool m_isRunning = false;
    };

    class CommandWatertightMesh : public Command
    {
        Q_OBJECT
    public:
        static constexpr std::string_view Name = "watertight-mesh";
        explicit CommandWatertightMesh(IAppContext* context);
        void execute() override;

    private:
        bool m_isRunning = false;
    };
    // ── Add this block inside the Mayo namespace in commands_koshika.h ──────────
//
//  #include "TriToQuad.h"   <-- put this near the other #includes at top of .h
//
// ---------------------------------------------------------------------------

    class CommandTriToQuad : public Command
    {
    public:
        static constexpr std::string_view Name = "tri-to-quad";
        explicit CommandTriToQuad(IAppContext* context);
        void execute() override;
     
    private:
        bool m_isRunning = false;
    };

} // namespace Mayo
