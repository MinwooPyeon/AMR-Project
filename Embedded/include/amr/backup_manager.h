#pragma once
#include <string>
#include <vector>

namespace amr {

class BackupManager {
public:
    explicit BackupManager(const std::string& backupDir);
    void save(const std::string& filename, const std::string& content);
    std::vector<std::string> list();
    void remove(const std::string& filename);

private:
    std::string dir_;
};

}
