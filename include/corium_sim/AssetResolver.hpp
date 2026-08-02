#pragma once

// =============================================================================
// Corium SimLab — Asset Path Resolver
// Resolves relative asset paths to absolute paths using the project root
// directory determined at compile-time via CMake.
// =============================================================================

#include <string>
#include <filesystem>
#include <vector>

namespace corium_sim {

/// @brief Resolves asset file paths relative to the project root directory.
/// Uses CORIUM_SIMLAB_ROOT_DIR (set via CMake compile definition) to locate
/// assets regardless of the current working directory at runtime.
class AssetResolver {
public:
    /// @brief Resolve a relative asset path to an existing absolute file path.
    /// @param relativePath  Path such as "assets/models/sample_robot.obj"
    /// @return Absolute path to the file if found, or the original path if resolution fails.
    [[nodiscard]] static std::string resolve(const std::string& relativePath)
    {
        namespace fs = std::filesystem;

        // 1. If already absolute and exists, return as-is
        if (fs::path(relativePath).is_absolute() && fs::exists(relativePath)) {
            return relativePath;
        }

        // 2. Try relative to compile-time project root
        std::string projectRoot = getProjectRoot();
        if (!projectRoot.empty()) {
            fs::path candidate = fs::path(projectRoot) / relativePath;
            if (fs::exists(candidate)) {
                return candidate.string();
            }
        }

        // 3. Try relative to CWD (original behavior)
        if (fs::exists(relativePath)) {
            return relativePath;
        }

        // 4. Try walking up from CWD to find the asset
        fs::path cwd = fs::current_path();
        for (int depth = 1; depth <= 5; ++depth) {
            std::string prefix;
            for (int i = 0; i < depth; ++i) prefix += "../";
            fs::path candidate = cwd / prefix / relativePath;
            if (fs::exists(candidate)) {
                return fs::canonical(candidate).string();
            }
        }

        // 5. Fallback: return original (will trigger error in caller)
        return relativePath;
    }

    /// @brief Get the compile-time project root directory.
    [[nodiscard]] static std::string getProjectRoot()
    {
#ifdef CORIUM_SIMLAB_ROOT_DIR
        return CORIUM_SIMLAB_ROOT_DIR;
#else
        return {};
#endif
    }
};

} // namespace corium_sim
