#include "amr/backup_manager.h"
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;
namespace amr {

BackupManager::BackupManager(const std::string& backupDir) : dir_(backupDir) {
    if (!fs::exists(dir_)) fs::create_directories(dir_);
}

void BackupManager::save(const std::string& filename, const std::string& content) {
    std::ofstream ofs(dir_ + "/" + filename);
    ofs << content;
}

std::vector<std::string> BackupManager::list() {
    std::vector<std::string> out;
    for (const auto& f : fs::directory_iterator(dir_))
        if (fs::is_regular_file(f)) out.push_back(f.path().filename().string());
    return out;
}

void BackupManager::remove(const std::string& filename) {
    fs::remove(dir_ + "/" + filename);
}

}
